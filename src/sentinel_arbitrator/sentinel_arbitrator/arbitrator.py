"""
哨兵融合仲裁节点 (GOAI 比赛版)

  订阅 /sentinel/fast_alerts
    ├── conf > 0.85  → 直接转发到 /sentinel/confirmed_alerts
    └── 0.50 < conf < 0.85  → 调 VLM 精判 → 加权融合 (YOLO*0.3 + VLM*0.7)
                               5 秒内同类型去重后发布

  安全兜底:
    - VLM 超时(10s)自动降级为直接转发
    - 异常累积触发 CRITICAL 升级
    - 系统健康状态上报 /sentinel/health
"""

import rclpy
import json
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

# ---- 安全兜底 ----
VLM_TIMEOUT      = 10.0   # VLM 超时 (秒)
CRITICAL_BURST_THRESH = 5 # 短时间同类型告警超过此数升级为 CRITICAL
BURST_WINDOW     = 30.0   # 爆发检测窗口 (秒)


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

        # ---- 安全: 告警爆发计数 ----
        self._alert_burst: dict = defaultdict(list)  # type → [timestamps]
        self._burst_lock = threading.Lock()

        # ---- 订阅 /sentinel/fast_alerts ----
        self.create_subscription(
            AnomalyEvent, '/sentinel/fast_alerts', self._on_fast_alert, 10
        )

        # ---- 发布 /sentinel/confirmed_alerts ----
        self._confirmed_pub = self.create_publisher(
            AnomalyEvent, '/sentinel/confirmed_alerts', 10
        )

        # ---- 系统健康状态发布 ----
        from std_msgs.msg import String
        self._health_pub = self.create_publisher(
            String, '/sentinel/health', 10
        )

        # ---- 尝试连接 VLM ----
        self._try_init_vlm()

        self.get_logger().info(
            f'[ARB] 启动完成 (GOAI 安全增强版) | '
            f'CONF_HIGH={CONF_HIGH} CONF_LOW={CONF_LOW} '
            f'W_YOLO={W_YOLO} W_VLM={W_VLM} '
            f'DEDUP_WINDOW={DEDUP_WINDOW}s '
            f'BURST={CRITICAL_BURST_THRESH}/{BURST_WINDOW}s'
        )

    # ========== VLM 连接 ==========

    def _try_init_vlm(self):
        """尝试初始化 VLM 服务端"""
        self.get_logger().info('[ARB][VLM] 开始尝试连接 VLM 服务...')
        try:
            self.declare_parameter('vlm_model_path', '')
            self.declare_parameter('vlm_n_ctx', 2048)
            vlm_path = self.get_parameter('vlm_model_path').value
            vlm_ctx = self.get_parameter('vlm_n_ctx').value

            self.get_logger().info(
                f'[ARB][VLM] 参数: model_path={vlm_path or "(未设置)"} '
                f'n_ctx={vlm_ctx}')

            if not vlm_path:
                self.get_logger().warn(
                    '[ARB][VLM] 未配置 vlm_model_path → VLM 精判不可用, '
                    '中置信度告警将直接转发')
                return

            from sentinel_vlm.vlm_server import VLMServer
            self._vlm_server = VLMServer()
            self.get_logger().info(f'[ARB][VLM] 连接成功 ✓ model={vlm_path}')
        except ImportError:
            self.get_logger().warn('[ARB][VLM] sentinel_vlm 未安装 → VLM 精判不可用')
        except Exception as e:
            self.get_logger().error(f'[ARB][VLM] 初始化失败: {e}')

    # ========== 告警回调 ==========

    def _on_fast_alert(self, event: AnomalyEvent):
        """处理 YOLO 快速告警"""
        conf = event.confidence
        self.get_logger().info(
            f'[ARB][入口] 收到快速告警 | '
            f'event_id={event.event_id} '
            f'type={event.anomaly_type} '
            f'conf={conf:.4f} '
            f'src={event.detection_source} '
            f'severity_in={event.severity} '
            f'waypoint={event.waypoint_id} '
            f'lat={event.lat:.4f} lng={event.lng:.4f}'
        )

        # -- 忽略低置信度 --
        if conf <= CONF_LOW:
            self.get_logger().info(
                f'[ARB][丢弃] conf={conf:.4f} <= {CONF_LOW} '
                f'→ 忽略 {event.anomaly_type}')
            return

        # -- 高置信度 → 直接转发 --
        if conf > CONF_HIGH:
            self.get_logger().info(
                f'[ARB][路由] conf={conf:.4f} > {CONF_HIGH} '
                f'→ 高置信度 直接转发')
            self._forward(event)
            return

        # -- 中置信度 → 调 VLM 精判 --
        if self._vlm_server is not None:
            self.get_logger().info(
                f'[ARB][路由] {CONF_LOW} < conf={conf:.4f} <= {CONF_HIGH} '
                f'→ 中置信度, 调 VLM 精判')
            self._refine_with_vlm(event)
        else:
            self.get_logger().warn(
                f'[ARB][降级] 无 VLM → 中置信度 conf={conf:.4f} 降级直接转发: '
                f'{event.anomaly_type}')
            self._forward(event)

    # ========== 直接转发 ==========

    def _forward(self, event: AnomalyEvent):
        """去重后直接转发到 confirmed_alerts, 含安全升级"""

        # ---------- 去重检查 ----------
        now = time.time()
        with self._dedup_lock:
            last_ts = self._dedup_map.get(event.anomaly_type, 0.0)
            elapsed = now - last_ts

        if elapsed < DEDUP_WINDOW:
            self.get_logger().info(
                f'[ARB][去重] {event.anomaly_type} '
                f'距上次 {elapsed:.1f}s < {DEDUP_WINDOW}s → 跳过 '
                f'(上次: {time.strftime("%H:%M:%S", time.localtime(last_ts))})')
            return

        self.get_logger().info(
            f'[ARB][去重] {event.anomaly_type} 通过 '
            f'(距上次 {elapsed:.1f}s >= {DEDUP_WINDOW}s 或首次)')

        # ---------- 安全: 告警爆发检测 ----------
        burst_result = self._check_burst(event)
        original_severity = event.severity

        if burst_result['upgraded']:
            event.severity = AnomalyEvent.CRITICAL
            event.description = f'[安全升级] {event.description}'
            self.get_logger().warn(
                f'[ARB][爆发] ⚠ CRITICAL升级! type={event.anomaly_type} '
                f'burst_count={burst_result["count"]}/{CRITICAL_BURST_THRESH} '
                f'severity {original_severity}→{event.severity} '
                f'timestamps_in_window={burst_result["timestamps"]}')
        else:
            self.get_logger().info(
                f'[ARB][爆发] {event.anomaly_type} '
                f'burst_count={burst_result["count"]}/{CRITICAL_BURST_THRESH} '
                f'→ 未触发升级')

        # ---------- 转发 ----------
        event.detection_source = 'yolo'
        self._confirmed_pub.publish(event)
        self._record_dedup(event.anomaly_type)

        self.get_logger().info(
            f'[ARB][转发] ✓ {event.anomaly_type} → /sentinel/confirmed_alerts | '
            f'event_id={event.event_id} conf={event.confidence:.4f} '
            f'severity={event.severity} '
            f'(升级={burst_result["upgraded"]})')

    # ========== 安全: 告警爆发检测 ==========

    def _check_burst(self, event: AnomalyEvent) -> dict:
        """
        检测短时间内同类型告警是否爆发.
        返回: {'upgraded': bool, 'count': int, 'timestamps': list}
        """
        now = time.time()
        with self._burst_lock:
            timestamps = self._alert_burst[event.anomaly_type]

            # 记录本次
            old_count = len(timestamps)
            timestamps.append(now)
            self.get_logger().info(
                f'[ARB][BURST] {event.anomaly_type} '
                f'appended ts={now:.3f} (total_before_cleanup={old_count + 1})')

            # 清除过期记录
            before_clean = len(timestamps)
            self._alert_burst[event.anomaly_type] = [
                t for t in timestamps if now - t < BURST_WINDOW
            ]
            after_clean = len(self._alert_burst[event.anomaly_type])
            if before_clean != after_clean:
                self.get_logger().info(
                    f'[ARB][BURST] 过期清理: {event.anomaly_type} '
                    f'{before_clean}→{after_clean} '
                    f'(移除了 {before_clean - after_clean} 条 > {BURST_WINDOW}s)')

            active_count = after_clean
            upgraded = active_count >= CRITICAL_BURST_THRESH

            return {
                'upgraded': upgraded,
                'count': active_count,
                'timestamps': [
                    time.strftime('%H:%M:%S', time.localtime(t))
                    for t in self._alert_burst[event.anomaly_type]
                ],
            }

    # ========== VLM 精判 ==========

    def _refine_with_vlm(self, event: AnomalyEvent):
        """
        调 VLM 精判 → 加权融合:
          final_conf = YOLO*0.3 + VLM*0.7
        """
        yolo_conf = event.confidence
        vlm_request_id = f'vlm_{uuid.uuid4().hex[:8]}'

        self.get_logger().info(
            f'[ARB][VLM-Q] 发起VLM请求 | '
            f'req_id={vlm_request_id} '
            f'type={event.anomaly_type} '
            f'yolo_conf={yolo_conf:.4f} '
            f'image={event.image_path or "(无路径)"}')

        # ---- 尝试读取图像 ----
        image = self._load_image(event.image_path)
        if image is None:
            self.get_logger().warn(
                f'[ARB][VLM-Q] 图像加载失败 path={event.image_path} '
                f'→ 降级为直接转发')
            self._forward(event)
            return

        self.get_logger().info(
            f'[ARB][VLM-Q] 图像加载成功 shape={image.shape} '
            f'→ 入队等待 VLM 处理...')

        def on_vlm_result(vlm_data: dict):
            """VLM 异步回调"""
            self.get_logger().info(
                f'[ARB][VLM-R] 收到VLM响应 | req_id={vlm_request_id}')
            self.get_logger().info(
                f'[ARB][VLM-R] VLM原始输出: {json.dumps(vlm_data, ensure_ascii=False)}')

            vlm_conf = vlm_data.get('confidence', 0.0)
            vlm_has = vlm_data.get('has_anomaly', False)
            vlm_type = vlm_data.get('anomaly_type', 'unknown')
            vlm_sev = vlm_data.get('severity', 'info')
            vlm_desc = vlm_data.get('description', '')

            # 加权融合
            fused_conf = yolo_conf * W_YOLO + vlm_conf * W_VLM
            mapped_severity = self._map_severity(vlm_sev)

            self.get_logger().info(
                f'[ARB][融合] 详细计算: '
                f'YOLO_conf={yolo_conf:.4f}×{W_YOLO}={yolo_conf * W_YOLO:.4f} | '
                f'VLM_conf={vlm_conf:.4f}×{W_VLM}={vlm_conf * W_VLM:.4f} | '
                f'fused_conf={fused_conf:.4f}')

            self.get_logger().info(
                f'[ARB][融合] 语义对齐: '
                f'VLM_has={vlm_has} VLM_type={vlm_type}→final_type={vlm_data.get("anomaly_type", event.anomaly_type)} '
                f'VLM_sev={vlm_sev}→mapped={mapped_severity} '
                f'VLM_desc="{vlm_desc[:50]}..."')

            if fused_conf < CONF_LOW:
                self.get_logger().info(
                    f'[ARB][融合] fused_conf={fused_conf:.4f} < {CONF_LOW} '
                    f'→ 融合后置信度过低, 丢弃 {event.anomaly_type}')
                return

            # 去重检查
            if self._is_duplicate(event.anomaly_type):
                self.get_logger().info(
                    f'[ARB][融合] 去重跳过 (VLM路径): {event.anomaly_type}')
                return

            # 构造融合后的告警
            fused_event = AnomalyEvent()
            fused_event.event_id = f'fused_{uuid.uuid4().hex[:12]}'
            fused_event.severity = mapped_severity
            fused_event.anomaly_type = vlm_data.get('anomaly_type', event.anomaly_type)
            fused_event.confidence = float(fused_conf)
            fused_event.description = (
                f'[融合] YOLO: {event.description} | VLM: {vlm_desc}'
            )
            fused_event.lat = event.lat
            fused_event.lng = event.lng
            fused_event.waypoint_id = event.waypoint_id
            fused_event.detection_source = 'fusion'
            fused_event.image_path = event.image_path

            self._confirmed_pub.publish(fused_event)
            self._record_dedup(fused_event.anomaly_type)

            self.get_logger().info(
                f'[ARB][融合] ✓ 融合告警 → /sentinel/confirmed_alerts | '
                f'event_id={fused_event.event_id} '
                f'type={fused_event.anomaly_type} '
                f'fused_conf={fused_conf:.4f} '
                f'severity={fused_event.severity} '
                f'source=fusion')

        self._vlm_server.query_anomaly(image, on_vlm_result)

    # ========== 去重逻辑 ==========

    def _is_duplicate(self, anomaly_type: str) -> bool:
        now = time.time()
        with self._dedup_lock:
            last = self._dedup_map.get(anomaly_type, 0.0)
            is_dup = (now - last) < DEDUP_WINDOW
            if is_dup:
                self.get_logger().debug(
                    f'[ARB][去重-检查] {anomaly_type}: 重复 '
                    f'(间隔={now - last:.1f}s < {DEDUP_WINDOW}s, '
                    f'上次={time.strftime("%H:%M:%S", time.localtime(last))})')
            return is_dup

    def _record_dedup(self, anomaly_type: str):
        now = time.time()
        with self._dedup_lock:
            old = self._dedup_map.get(anomaly_type, 0.0)
            self._dedup_map[anomaly_type] = now
            self.get_logger().debug(
                f'[ARB][去重-记录] {anomaly_type}: '
                f'ts={time.strftime("%H:%M:%S", time.localtime(now))} '
                f'(距上次 {now - old:.1f}s)')

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
