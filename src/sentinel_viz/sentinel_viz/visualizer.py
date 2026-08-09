"""
哨兵感知融合可视化节点

  订阅 /sentinel/fast_alerts 和 /sentinel/confirmed_alerts,
  以 RViz MarkerArray 散点+射线形式展示感知输入与融合结果.

  颜色映射:
    CRITICAL → 红色球体 (大)
    WARNING  → 橙色球体 (中)
    INFO     → 黄色球体 (小)
  射线: 机器人→检测位置的绿色虚线
"""

import rclpy
import math
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sentinel_interfaces.msg import AnomalyEvent


# ---- 颜色/尺寸常量 ----
SEVERITY_COLORS = {
    0: (1.0, 1.0, 0.0, 0.7),   # INFO    黄色
    1: (1.0, 0.5, 0.0, 0.8),   # WARNING 橙色
    2: (1.0, 0.0, 0.0, 0.9),   # CRITICAL 红色
}
SEVERITY_SCALES = {0: 0.5, 1: 0.7, 2: 1.0}
SOURCE_COLORS = {
    'yolo':   (0.3, 0.8, 1.0, 0.6),  # 浅蓝
    'vlm':    (0.8, 0.3, 1.0, 0.6),  # 紫色
    'fusion': (0.0, 1.0, 0.5, 0.8),  # 绿色
}

ROBOT_POS = (0.0, 0.0, 0.0)  # 机器人位置 (假设原点)

class SentinelVisualizer(Node):
    """感知融合可视化"""

    def __init__(self):
        super().__init__('sentinel_viz')

        # ---- 发布 ----
        self._marker_pub = self.create_publisher(
            MarkerArray, '/sentinel/viz/markers', 10
        )

        # ---- 订阅 ----
        self.create_subscription(
            AnomalyEvent, '/sentinel/fast_alerts', self._on_fast_alert, 100
        )
        self.create_subscription(
            AnomalyEvent, '/sentinel/confirmed_alerts', self._on_confirmed, 100
        )

        # ---- 状态 ----
        self._fast_detections: list[dict] = []   # YOLO 原始检测
        self._confirmed_alerts: list[dict] = []  # 融合后确认告警
        self._marker_id = 0

        # 定时刷新 (2Hz)
        self._timer = self.create_timer(0.5, self._publish_markers)
        self.get_logger().info('SentinelVisualizer 启动 (RViz MarkerArray)')

    # ========== 订阅回调 ==========

    def _on_fast_alert(self, event: AnomalyEvent):
        """收集 YOLO 原始检测"""
        if not self._is_valid_position(event):
            return
        entry = {
            'event_id': event.event_id,
            'type': event.anomaly_type,
            'severity': event.severity,
            'conf': event.confidence,
            'lat': event.lat,
            'lng': event.lng,
            'source': 'yolo',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        }
        self._fast_detections.append(entry)
        # 只保留最近 50 条
        if len(self._fast_detections) > 50:
            self._fast_detections = self._fast_detections[-50:]

    def _on_confirmed(self, event: AnomalyEvent):
        """收集确认告警"""
        if not self._is_valid_position(event):
            return
        entry = {
            'event_id': event.event_id,
            'type': event.anomaly_type,
            'severity': event.severity,
            'conf': event.confidence,
            'lat': event.lat,
            'lng': event.lng,
            'source': event.detection_source,
            'desc': event.description[:60],
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        }
        self._confirmed_alerts.append(entry)
        if len(self._confirmed_alerts) > 50:
            self._confirmed_alerts = self._confirmed_alerts[-50:]

    # ========== 发布标记 ==========

    def _publish_markers(self):
        """构建并发布 MarkerArray"""
        now = self.get_clock().now().nanoseconds / 1e9
        marker_array = MarkerArray()
        self._marker_id = 0

        # -- 1. 机器人位置 (白色小球 + 坐标轴) --
        self._make_sphere(marker_array, 0, 0, 0,
                          color=(1, 1, 1, 0.5), scale=0.3,
                          ns='robot')

        # -- 2. YOLO 快速检测 → 浅蓝半透明球 --
        for d in self._recent(self._fast_detections, now, 30):
            x, y = self._latlng_to_xy(d['lat'], d['lng'])
            r, g, b, a = SOURCE_COLORS['yolo']
            self._make_sphere(marker_array, x, y, 0,
                              color=(r, g, b, a), scale=0.3,
                              ns='fast_detections')
            # 射线
            self._make_ray(marker_array, 0, 0, x, y,
                           color=(r, g, b, 0.3), ns='fast_rays')

        # -- 3. 确认告警 → 按severity着色球体 --
        for d in self._recent(self._confirmed_alerts, now, 60):
            x, y = self._latlng_to_xy(d['lat'], d['lng'])
            r, g, b, a = SEVERITY_COLORS.get(d['severity'], (0.5, 0.5, 0.5, 0.5))
            s = SEVERITY_SCALES.get(d['severity'], 0.5)
            # 主球
            self._make_sphere(marker_array, x, y, 0,
                              color=(r, g, b, a), scale=0.4 * s,
                              ns='confirmed')
            # 射线 (按来源颜色)
            src_r, src_g, src_b, src_a = SOURCE_COLORS.get(
                d['source'], (0.5, 0.5, 0.5, 0.5))
            self._make_ray(marker_array, 0, 0, x, y,
                           color=(src_r, src_g, src_b, 0.4), ns='confirmed_rays')
            # 文字标签
            tag = f"{d['type'][:8]}({d['conf']:.2f})"
            self._make_text(marker_array, x, y, 0.6, tag,
                            color=(1, 1, 1, 0.9), ns='labels')

        # -- 4. 融合结果 → 绿色大球 + 黄色射线 --
        fusion_alerts = [d for d in self._confirmed_alerts if d['source'] == 'fusion']
        for d in self._recent(fusion_alerts, now, 60):
            x, y = self._latlng_to_xy(d['lat'], d['lng'])
            r, g, b, a = SOURCE_COLORS['fusion']
            self._make_sphere(marker_array, x, y, 0.1,
                              color=(r, g, b, a), scale=0.5,
                              ns='fusion')
            self._make_ray(marker_array, 0, 0, x, y,
                           color=(r, g, b, 0.6), ns='fusion_rays')

        # 发布
        self._marker_pub.publish(marker_array)

    # ========== 标记构建方法 ==========

    def _make_sphere(self, marker_array: MarkerArray, x, y, z,
                     color, scale, ns):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = self._marker_id
        self._marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=z)
        marker.pose.orientation.w = 1.0
        marker.scale = Point(x=scale, y=scale, z=scale)
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        marker.lifetime.sec = 5
        marker_array.markers.append(marker)

    def _make_ray(self, marker_array: MarkerArray, x1, y1, x2, y2,
                  color, ns):
        """从机器人到检测点的射线"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = self._marker_id
        self._marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale = Point(x=0.05, y=0.0, z=0.0)
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        marker.pose.orientation.w = 1.0
        marker.points = [
            Point(x=x1, y=y1, z=0.0),
            Point(x=x2, y=y2, z=0.0),
        ]
        marker.lifetime.sec = 3
        marker_array.markers.append(marker)

    def _make_text(self, marker_array: MarkerArray, x, y, z,
                   text, color, ns):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = self._marker_id
        self._marker_id += 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=z)
        marker.pose.orientation.w = 1.0
        marker.scale = Point(x=0.3, y=0.3, z=0.3)
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        marker.text = text
        marker.lifetime.sec = 4
        marker_array.markers.append(marker)

    # ========== 工具 ==========

    @staticmethod
    def _recent(items, now, max_age):
        """过滤最近 max_age 秒内的条目"""
        return [d for d in items if now - d['timestamp'] < max_age]

    @staticmethod
    def _latlng_to_xy(lat, lng):
        """
        将经纬度近似转为本地 XY (米).
        以预设巡检点中心为原点.
        """
        REF_LAT = 30.3185
        REF_LNG = 120.0695
        # 1度 ≈ 111320 m (纬度), 1度 ≈ 111320*cos(lat) m (经度)
        dlat = (lat - REF_LAT) * 111320.0
        dlng = (lng - REF_LNG) * 111320.0 * math.cos(math.radians(REF_LAT))
        return dlng, dlat  # x=东, y=北

    @staticmethod
    def _is_valid_position(event):
        return not (event.lat == 0.0 and event.lng == 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = SentinelVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
