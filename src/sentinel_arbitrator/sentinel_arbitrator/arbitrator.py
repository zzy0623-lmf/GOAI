"""
哨兵融合仲裁节点

  订阅 /sentinel/fast_alerts
    ├── conf > 0.85  → 直接转发到 /sentinel/confirmed_alerts
    └── 0.50 < conf < 0.85  → 调 VLM 精判 → 加权融合 (YOLO*0.3 + VLM*0.7)
                               5 秒内同类型去重后发布
"""

import rclpy
import uuid
import time
import threading
from collections import defaultdict
from rclpy.node import Node
from sentinel_interfaces.msg import AnomalyEvent

# ---- 权重 ----
W_YOLO = 0.3
W_VLM  = 0.7

# ---- 置信度阈值 ----
CONF_HIGH   = 0.85   # 直接转发
CONF_LOW    = 0.50   # 低于此值忽略

# ---- 去重窗口 (秒) ----
DEDUP_WINDOW = 5.0


class Arbitrator(Node):
    """融合仲裁节点"""

    def __init__(self):
        super().__init__('arbitrator')

        # VLM 客户端 (延迟加载)
        self._vlm_server = None
        self._vlm_lock = threading.Lock()

        # ---- 去重记录: {anomaly_type: last_timestamp} ----
        self._dedup_map = {}               # type → timestamp
        self._dedup_lock = threading.Lock()

        # ---- 订阅 /sentinel/fast_alerts ----
        self.create_subscription(
            AnomalyEvent, '/sentinel/fast_alerts', self._on_fast_alert, 10
        )

        # ---- 发布 /sentinel/confirmed_alerts ----
        self._confirmed_pub = self.create_publisher(
            AnomalyEvent, '/sentinel/confirmed_alerts', 10
        )

        # ---- 尝试连接 VLM ----
        self._try_init_vlm()

        self.get_logger().info('Arbitrator 启动完成')

    # ========== VLM 连接 ==========

    def _try_init_vlm(self):
        """尝试初始化 VLM 服务端"""
        try:
            # 读取 VLM 参数
            self.declare_parameter('vlm_model_path', '')
            self.declare_parameter('vlm_n_ctx', 2048)
            vlm_path = self.get_parameter('vlm_model_path').value
            vlm_ctx = self.get_parameter('vlm_n_ctx').value

            if not vlm_path:
                self.get_logger().warn('未配置 vlm_model_path, VLM 精判不可用, '
                                       '所有告警将直接转发 (conf>0.5 即可)')
                return

            from sentinel_vlm.vlm_server import VLMServer
            self._vlm_server = VLMServer()
            self.get_logger().info(f'VLM 已连接: {vlm_path}')
        except ImportError:
            self.get_logger().warn('sentinel_vlm 未安装, VLM 精判不可用')
        except Exception as e:
            self.get_logger().error(f'VLM 初始化失败: {e}')

    # ========== 告警回调 ==========

    def _on_fast_alert(self, event: AnomalyEvent):
        """处理 YOLO 快速告警"""
        conf = event.confidence

        self.get_logger().info(
            f'收到快速告警: type={event.anomaly_type} conf={conf:.2f} '
            f'id={event.event_id}')

        # -- 忽略低置信度 --
        if conf <= CONF_LOW:
            self.get_logger().debug(f'忽略 (conf<=0.50): {event.anomaly_type}')
            return

        # -- 高置信度 → 直接转发 --
        if conf > CONF_HIGH:
            self._forward(event)
            return

        # -- 中置信度 → 调 VLM 精判 --
        if self._vlm_server is not None:
            self._refine_with_vlm(event)
        else:
            # 无 VLM 时, 中置信度降级直接转发
            self.get_logger().warn(
                f'无 VLM, 中置信度直接转发: {event.anomaly_type}')
            self._forward(event)

    # ========== 直接转发 ==========

    def _forward(self, event: AnomalyEvent):
        """去重后直接转发到 confirmed_alerts"""
        if self._is_duplicate(event.anomaly_type):
            self.get_logger().info(
                f'去重跳过: {event.anomaly_type} (5秒内已发)')
            return

        event.detection_source = 'yolo'
        self._confirmed_pub.publish(event)
        self._record_dedup(event.anomaly_type)
        self.get_logger().info(
            f'直接转发 → confirmed: {event.anomaly_type} conf={event.confidence:.2f}')

    # ========== VLM 精判 ==========

    def _refine_with_vlm(self, event: AnomalyEvent):
        """
        调 VLM 精判 → 加权融合:
          final_conf = YOLO*0.3 + VLM*0.7
        """
        import numpy as np

        # ---- 尝试读取图像 ----
        image = self._load_image(event.image_path)
        if image is None:
            self.get_logger().warn(
                f'无法加载图像 {event.image_path}, 降级为直接转发')
            self._forward(event)
            return

        yolo_conf = event.confidence

        def on_vlm_result(vlm_data: dict):
            vlm_conf = vlm_data.get('confidence', 0.0)

            # 加权融合
            fused_conf = yolo_conf * W_YOLO + vlm_conf * W_VLM

            self.get_logger().info(
                f'VLM 融合: YOLO={yolo_conf:.2f}*{W_YOLO} + '
                f'VLM={vlm_conf:.2f}*{W_VLM} = {fused_conf:.2f}')

            if fused_conf < CONF_LOW:
                self.get_logger().info(f'融合后置信度过低, 丢弃: {event.anomaly_type}')
                return

            if self._is_duplicate(event.anomaly_type):
                self.get_logger().info(f'去重跳过 (VLM): {event.anomaly_type}')
                return

            # 构造融合后的告警
            fused_event = AnomalyEvent()
            fused_event.event_id = f'fused_{uuid.uuid4().hex[:12]}'
            fused_event.severity = self._map_severity(vlm_data.get('severity', 'info'))
            fused_event.anomaly_type = vlm_data.get('anomaly_type', event.anomaly_type)
            fused_event.confidence = float(fused_conf)
            fused_event.description = (
                f'[融合] YOLO: {event.description} | VLM: '
                f'{vlm_data.get("description", "")}'
            )
            fused_event.lat = event.lat
            fused_event.lng = event.lng
            fused_event.waypoint_id = event.waypoint_id
            fused_event.detection_source = 'fusion'
            fused_event.image_path = event.image_path

            self._confirmed_pub.publish(fused_event)
            self._record_dedup(fused_event.anomaly_type)
            self.get_logger().info(
                f'融合告警 → confirmed: {fused_event.anomaly_type} '
                f'conf={fused_conf:.2f} severity={fused_event.severity}')

        self._vlm_server.query_anomaly(image, on_vlm_result)

    # ========== 去重逻辑 ==========

    def _is_duplicate(self, anomaly_type: str) -> bool:
        now = time.time()
        with self._dedup_lock:
            last = self._dedup_map.get(anomaly_type, 0.0)
            return (now - last) < DEDUP_WINDOW

    def _record_dedup(self, anomaly_type: str):
        with self._dedup_lock:
            self._dedup_map[anomaly_type] = time.time()

    # ========== 工具方法 ==========

    @staticmethod
    def _load_image(path: str):
        """加载图像为 numpy 数组"""
        if not path:
            return None
        try:
            import cv2
            img = cv2.imread(path)
            if img is None:
                return None
            return img
        except Exception:
            return None

    @staticmethod
    def _map_severity(vlm_severity: str) -> int:
        """VLM severity 字符串 → AnomalyEvent 常量"""
        mapping = {
            'critical': AnomalyEvent.CRITICAL,
            'warning':  AnomalyEvent.WARNING,
            'info':     AnomalyEvent.INFO,
        }
        return mapping.get(vlm_severity.lower(), AnomalyEvent.INFO)


def main(args=None):
    rclpy.init(args=args)
    node = Arbitrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
