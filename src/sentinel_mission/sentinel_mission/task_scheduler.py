"""
哨兵任务调度节点

  预设 5 个巡检点.
  订阅 /sentinel/confirmed_alerts, 告警位置 50m 内的巡检点:
    priority = 1, dwell_time = min(翻倍, 60s).
  get_next_plan() 服务返回按 (priority DESC, id ASC) 排序的巡检计划.
"""

import rclpy
import json
import math
import threading
from typing import List
from rclpy.node import Node
from std_srvs.srv import Trigger
from sentinel_interfaces.msg import AnomalyEvent

# ==================== 预设巡检点 ====================
# 坐标以某园区为参考 (上海张江)

DEFAULT_WAYPOINTS = [
    {'id': 1, 'name': '正门岗亭',     'lat': 31.2045, 'lng': 121.5850, 'priority': 3, 'dwell': 10},
    {'id': 2, 'name': '研发楼A栋',    'lat': 31.2050, 'lng': 121.5860, 'priority': 3, 'dwell': 10},
    {'id': 3, 'name': '数据中心',     'lat': 31.2040, 'lng': 121.5870, 'priority': 5, 'dwell': 15},
    {'id': 4, 'name': '停车场',       'lat': 31.2035, 'lng': 121.5845, 'priority': 3, 'dwell': 10},
    {'id': 5, 'name': '配电站',       'lat': 31.2055, 'lng': 121.5855, 'priority': 5, 'dwell': 15},
]

ALERT_RADIUS_M  = 50.0    # 告警影响半径
ALERT_PRIORITY  = 1       # 告警时设为此值
MAX_DWELL       = 60      # dwell 上限 (秒)


class TaskScheduler(Node):
    """任务调度节点"""

    def __init__(self):
        super().__init__('task_scheduler')

        # ---- 加载巡检点 (支持参数覆盖) ----
        self.declare_parameter('waypoints', json.dumps(DEFAULT_WAYPOINTS))
        wp_json = self.get_parameter('waypoints').value
        self._waypoints: list[dict] = json.loads(wp_json)
        self._lock = threading.Lock()

        self.get_logger().info(f'加载 {len(self._waypoints)} 个巡检点')

        # ---- 订阅告警 ----
        self.create_subscription(
            AnomalyEvent, '/sentinel/confirmed_alerts', self._on_alert, 10
        )

        # ---- 查询计划服务 ----
        self._srv = self.create_service(
            Trigger, '~/get_next_plan', self._on_get_next_plan
        )

        self.get_logger().info('TaskScheduler 启动完成')

    # ========== 告警处理 ==========

    def _on_alert(self, event: AnomalyEvent):
        """告警触发: 更新附近巡检点"""
        with self._lock:
            updated = []
            for wp in self._waypoints:
                dist = self._haversine(event.lat, event.lng, wp['lat'], wp['lng'])
                if dist < ALERT_RADIUS_M:
                    wp['priority'] = ALERT_PRIORITY
                    wp['dwell'] = min(wp['dwell'] * 2, MAX_DWELL)
                    updated.append(f"{wp['name']}({dist:.0f}m, dwell={wp['dwell']}s)")

            if updated:
                self.get_logger().info(
                    f'告警触发: {event.anomaly_type} → 调整巡检点: {", ".join(updated)}')

    # ========== 计划查询 ==========

    def _on_get_next_plan(self, req, resp):
        """返回按 (priority DESC, id ASC) 排序的巡检计划"""
        try:
            plan = self.get_next_plan()
            resp.success = True
            resp.message = json.dumps(plan, ensure_ascii=False)
            self.get_logger().info(f'返回巡检计划: {len(plan)} 个点')
        except Exception as e:
            resp.success = False
            resp.message = f'计划生成失败: {e}'
            self.get_logger().error(resp.message)
        return resp

    def get_next_plan(self) -> list[dict]:
        """
        返回排好序的巡检序列.
        排序: priority 降序 → id 升序
        """
        with self._lock:
            sorted_wps = sorted(self._waypoints, key=lambda w: (-w['priority'], w['id']))

        return [
            {
                'id': w['id'],
                'name': w['name'],
                'lat': w['lat'],
                'lng': w['lng'],
                'priority': w['priority'],
                'dwell': w['dwell'],
            }
            for w in sorted_wps
        ]

    # ========== 工具 ==========

    @staticmethod
    def _haversine(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
        """
        Haversine 公式计算两点距离 (米).
        """
        R = 6371000.0  # 地球半径 (米)
        dlat = math.radians(lat2 - lat1)
        dlng = math.radians(lng2 - lng1)
        a = (math.sin(dlat / 2) ** 2 +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(dlng / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c


def main(args=None):
    rclpy.init(args=args)
    node = TaskScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
