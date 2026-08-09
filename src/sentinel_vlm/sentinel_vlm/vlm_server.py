"""
哨兵 VLM 推理服务 (llama-cpp-python + GGUF)

提供三个查询接口：
  query_anomaly()   - priority=1 (异常检测, 最高优先级)
  query_scene()     - priority=2 (场景描述)
  query_chat()      - priority=3 (对话)

工作线程按优先级从队列取请求，调用 LLM 后通过 callback 返回结果。
"""

import rclpy
import re
import json
import uuid
import base64
import threading
from queue import PriorityQueue, Empty

import cv2
import numpy as np
from rclpy.node import Node

# ---------- Prompt 模板 ----------

PROMPT_ANOMALY = """你是一台园区巡逻机器人，正在分析摄像头画面。请判断图像中是否存在异常。

可能的异常类型: fire(火灾), intrusion(人员闯入), equipment_damage(设备损坏),
  spill(液体泄漏), obstruction(通道堵塞), vehicle_illegal(违规停车), other(其他)

请严格输出如下 JSON，不要包含任何其他文字:
{"has_anomaly": bool, "anomaly_type": "异常类型", "severity": "critical/warning/info", "description": "简短描述", "confidence": 0.0~1.0}

图像已通过视觉编码器处理。"""

PROMPT_SCENE = """你是一台园区巡逻机器人。请简要描述当前画面中的场景内容，包括主要物体、环境特征和值得注意的细节。
输出 JSON: {"scene": "场景描述", "objects": ["物体1", "物体2"], "risk_level": "low/medium/high"}"""

PROMPT_CHAT = """你是一台园区巡逻机器人助手。请回答用户的问题，回答应简短专业。"""


class VLMServer(Node):
    """VLM 推理服务器节点"""

    def __init__(self):
        super().__init__('vlm_server')

        # --- 参数 ---
        self.declare_parameter('model_path', '')
        self.declare_parameter('n_ctx', 2048)
        self.declare_parameter('mmproj_path', '')  # 多模态投影层路径 (如有)

        model_path = self.get_parameter('model_path').value
        n_ctx = self.get_parameter('n_ctx').value
        mmproj_path = self.get_parameter('mmproj_path').value

        if not model_path:
            self.get_logger().error('请通过 --ros-args -p model_path:=/path/to/model.gguf 指定模型路径')
            self.llm = None
        else:
            self.get_logger().info(f'加载 GGUF 模型: {model_path}')

            try:
                from llama_cpp import Llama
                kwargs = dict(
                    model_path=model_path,
                    n_ctx=n_ctx,
                    n_threads=4,
                    verbose=False,
                )
                if mmproj_path:
                    kwargs['mmproj'] = mmproj_path
                    self.get_logger().info(f'多模态投影: {mmproj_path}')

                self.llm = Llama(**kwargs)
                self.get_logger().info('模型加载成功')
            except ImportError:
                self.get_logger().error('请安装 llama-cpp-python: pip install llama-cpp-python')
                self.llm = None
            except Exception as e:
                self.get_logger().error(f'模型加载失败: {e}')
                self.llm = None

        # --- 优先级队列 ---
        # (priority, counter, request_id, image, callback, request_type)
        self._queue = PriorityQueue()
        self._counter = 0
        self._lock = threading.Lock()

        # --- 工作线程 ---
        self._worker = threading.Thread(target=self._process_loop, daemon=True)
        self._worker.start()

        self.get_logger().info('VLMServer 启动完成')

    # ========== 公共接口 ==========

    def query_anomaly(self, image: np.ndarray, callback) -> str:
        """
        异常检测 — priority=1 (最高)
        返回 request_id
        """
        return self._enqueue(1, image, callback, 'anomaly')

    def query_scene(self, image: np.ndarray, callback) -> str:
        """
        场景描述 — priority=2
        """
        return self._enqueue(2, image, callback, 'scene')

    def query_chat(self, text: str, callback) -> str:
        """
        对话 — priority=3 (最低)
        text 作为 image 参数传入 (复用通道)
        """
        return self._enqueue(3, text, callback, 'chat')

    # ========== 内部方法 ==========

    def _enqueue(self, priority: int, payload, callback, req_type: str) -> str:
        request_id = uuid.uuid4().hex[:12]
        with self._lock:
            self._counter += 1
            self._queue.put((priority, self._counter, request_id,
                             payload, callback, req_type))
            self.get_logger().info(f'入队 [{req_type}] pri={priority} id={request_id} '
                                   f'queue_size={self._queue.qsize()}')
        return request_id

    def _process_loop(self):
        """工作线程主循环"""
        while rclpy.ok():
            try:
                pri, _, rid, payload, callback, req_type = \
                    self._queue.get(timeout=2.0)
            except Empty:
                continue

            self.get_logger().info(f'处理 [{req_type}] id={rid}')

            try:
                if req_type == 'anomaly':
                    result = self._infer_anomaly(payload)
                elif req_type == 'scene':
                    result = self._infer_scene(payload)
                elif req_type == 'chat':
                    result = self._infer_chat(payload)
                else:
                    result = {'error': f'unknown type: {req_type}'}
            except Exception as e:
                self.get_logger().error(f'推理异常 [{rid}]: {e}')
                result = {'has_anomaly': False, 'anomaly_type': '',
                          'severity': 'info', 'description': f'推理失败: {e}',
                          'confidence': 0.0}

            try:
                callback(result)
            except Exception as e:
                self.get_logger().error(f'回调执行失败 [{rid}]: {e}')

    def _infer_anomaly(self, image: np.ndarray) -> dict:
        """异常检测推理"""
        if self.llm is None:
            return self._fallback_result()

        img_b64 = self._encode_image(image)

        try:
            # 优先尝试多模态 chat 接口
            output = self.llm.create_chat_completion(
                messages=[
                    {
                        'role': 'user',
                        'content': [
                            {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{img_b64}'}},
                            {'type': 'text', 'text': PROMPT_ANOMALY},
                        ],
                    }
                ],
                max_tokens=256,
                temperature=0.1,
            )
            text = output['choices'][0]['message']['content']
        except Exception:
            # 回退到纯文本 (模型不支持多模态时)
            prompt = f'{PROMPT_ANOMALY}\n[注意: 当前模型不支持图像输入，请基于常识判断]\nJSON:'
            output = self.llm(prompt, max_tokens=256, temperature=0.1, echo=False)
            text = output['choices'][0]['text']

        result = self._parse_json(text)
        self.get_logger().info(f'VLM anomaly: {json.dumps(result, ensure_ascii=False)}')
        return result

    def _infer_scene(self, image: np.ndarray) -> dict:
        """场景描述推理"""
        if self.llm is None:
            return {'scene': '模型未加载', 'objects': [], 'risk_level': 'low'}

        img_b64 = self._encode_image(image)
        try:
            output = self.llm.create_chat_completion(
                messages=[{
                    'role': 'user',
                    'content': [
                        {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{img_b64}'}},
                        {'type': 'text', 'text': PROMPT_SCENE},
                    ],
                }],
                max_tokens=256,
                temperature=0.3,
            )
            text = output['choices'][0]['message']['content']
        except Exception:
            output = self.llm(f'{PROMPT_SCENE}\nJSON:', max_tokens=256, temperature=0.3, echo=False)
            text = output['choices'][0]['text']

        return self._parse_json(text)

    def _infer_chat(self, text: str) -> dict:
        """对话推理"""
        if self.llm is None:
            return {'response': '模型未加载'}

        output = self.llm.create_chat_completion(
            messages=[
                {'role': 'system', 'content': PROMPT_CHAT},
                {'role': 'user', 'content': text},
            ],
            max_tokens=256,
            temperature=0.7,
        )
        reply = output['choices'][0]['message']['content']
        return {'response': reply}

    # ========== 工具 ==========

    @staticmethod
    def _encode_image(image: np.ndarray) -> str:
        """numpy 图像 → base64 JPEG"""
        _, buf = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return base64.b64encode(buf).decode('utf-8')

    @staticmethod
    def _parse_json(text: str) -> dict:
        """从 LLM 输出中提取 JSON"""
        text = text.strip()
        # 直接解析
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass
        # 提取 {...}
        match = re.search(r'\{[^{}]*\}', text)
        if match:
            try:
                return json.loads(match.group())
            except json.JSONDecodeError:
                pass
        # 提取含嵌套的 {...}
        match = re.search(r'\{.*\}', text, re.DOTALL)
        if match:
            try:
                return json.loads(match.group())
            except json.JSONDecodeError:
                pass
        return {'has_anomaly': False, 'anomaly_type': '', 'severity': 'info',
                'description': f'JSON解析失败: {text[:100]}', 'confidence': 0.0}

    @staticmethod
    def _fallback_result() -> dict:
        return {'has_anomaly': False, 'anomaly_type': '', 'severity': 'info',
                'description': 'VLM 模型未加载', 'confidence': 0.0}


def main(args=None):
    rclpy.init(args=args)
    node = VLMServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
