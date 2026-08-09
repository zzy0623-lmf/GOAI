"""
哨兵任务调度节点 (GOAI 比赛版)

  预设 5 个巡检点.
  订阅 /sentinel/confirmed_alerts, 告警位置 30m 内的巡检点:
    priority = 1, dwell_time = min(翻倍, 60s).
  get_next_plan() 服务返回按 (priority DESC, id ASC) 排序的巡检计划.
"""

import rclpy
import json
import math
import time
import threading
from typing import List
from rclpy.node import Node
from std_srvs.srv import Trigger
from sentinel_interfaces.msg import AnomalyEvent

# ==================== 预设巡检点 ====================
# 坐标: 杭州云谷中心 GOAI 比赛场地 (西湖区灯彩街1009号)

DEFAULT_WAYPOINTS = [
    {'id': 1, 'name': '园区主入口',    'lat': 30.3185, 'lng': 120.0690, 'priority': 3, 'dwell': 10},
    {'id': 2, 'name': '研发楼A栋',     'lat': 30.3190, 'lng': 120.0695, 'priority': 3, 'dwell': 10},
    {'id': 3, 'name': '数据中心/机房',  'lat': 30.3180, 'lng': 120.0700, 'priority': 5, 'dwell': 15},
    {'id': 4, 'name': '地下停车场入口',  'lat': 30.3175, 'lng': 120.0685, 'priority': 3, 'dwell': 10},
    {'id': 5, 'name': '配电站/设备区',   'lat': 30.3195, 'lng': 120.0695, 'priority': 5, 'dwell': 15},
]

ALERT_RADIUS_M  = 30.0    # 告警影响半径 (园区尺度适配)
ALERT_PRIORITY  = 1       # 告警时设为此值
MAX_DWELL       = 60      # dwell 上限 (秒)
ROBOT_MAX_SPEED = 8.0     # 山猫 S10 最高速度 m/s


class TaskScheduler(Node):
    """任务调度节点"""

    def __init__(self):
        super().__init__('task_scheduler')

        # ---- 加载巡检点 (支持参数覆盖) ----
        self.declare_parameter('waypoints', json.dumps(DEFAULT_WAYPOINTS))
        wp_json = self.get_parameter('waypoints').value
        self._waypoints: list[dict] = json.loads(wp_json)
        self._lock = threading.Lock()

        self.get_logger().info(f'[SCH] 加载 {len(self._waypoints)} 个巡检点:')
        for wp in self._waypoints:
            self.get_logger().info(
                f'[SCH]   点{wp["id"]}: {wp["name"]} '
                f'({wp["lat"]:.4f}, {wp["lng"]:.4f}) '
                f'initial_priority={wp["priority"]} initial_dwell={wp["dwell"]}s')

        # ---- 订阅告警 ----
        self.create_subscription(
            AnomalyEvent, '/sentinel/confirmed_alerts', self._on_alert, 10
        )

        # ---- 查询计划服务 ----
        self._srv = self.create_service(
            Trigger, '~/get_next_plan', self._on_get_next_plan
        )

        self.get_logger().info(
            f'[SCH] 启动完成 | '
            f'ALERT_RADIUS={ALERT_RADIUS_M}m '
            f'MAX_DWELL={MAX_DWELL}s '
            f'ROBOT_MAX_SPEED={ROBOT_MAX_SPEED}m/s')

    # ========== 告警处理 ==========

    def _on_alert(self, event: AnomalyEvent):
        """
        告警触发: 更新附近巡检点.
        distance < 30m → priority=1, dwell=min(dwell*2, 60s)
        """
        alert_lat = event.lat
        alert_lng = event.lng

        self.get_logger().info(
            f'[SCH][告警] 收到确认告警 | '
            f'event_id={event.event_id} '
            f'type={event.anomaly_type} '
            f'severity={event.severity} '
            f'conf={event.confidence:.4f} '
            f'src={event.detection_source} '
            f'alert_pos=({alert_lat:.6f}, {alert_lng:.6f}) '
            f'waypoint_from={event.waypoint_id}')

        with self._lock:
            updated = []
            skipped = []

            for wp in self._waypoints:
                dist = self._haversine(alert_lat, alert_lng, wp['lat'], wp['lng'])
                old_priority = wp['priority']
                old_dwell = wp['dwell']

                if dist < ALERT_RADIUS_M:
                    wp['priority'] = ALERT_PRIORITY
                    new_dwell = min(wp['dwell'] * 2, MAX_DWELL)
                    wp['dwell'] = new_dwell
                    updated.append({
                        'id': wp['id'],
                        'name': wp['name'],
                        'dist_m': round(dist, 1),
                        'priority': f'{old_priority}→{wp["priority"]}',
                        'dwell': f'{old_dwell}→{wp["dwell"]}s',
                    })
                else:
                    skipped.append({
                        'id': wp['id'],
                        'name': wp['name'],
                        'dist_m': round(dist, 1),
                    })

            # ---------- 详细日志 ----------
            self.get_logger().info(
                f'[SCH][告警] 巡检点距离计算完成 | '
                f'alert_type={event.anomaly_type} '
                f'alert_pos=({alert_lat:.6f}, {alert_lng:.6f})')

            for s in skipped:
                self.get_logger().info(
                    f'[SCH][告警]   点{s["id"]}: {s["name"]} '
                    f'dist={s["dist_m"]}m >= {ALERT_RADIUS_M}m → 不调整')

            if updated:
                self.get_logger().warn(
                    f'[SCH][告警] ⚡ {len(updated)} 个巡检点在告警影响范围内:')
                for u in updated:
                    self.get_logger().warn(
                        f'[SCH][告警]   id={u["id"]} {u["name"]} '
                        f'dist={u["dist_m"]}m < {ALERT_RADIUS_M}m '
                        f'→ priority={u["priority"]} dwell={u["dwell"]}')
            else:
                self.get_logger().info(
                    f'[SCH][告警] 告警位置 ({alert_lat:.6f}, {alert_lng:.6f}) '
                    f'{ALERT_RADIUS_M}m 内无巡检点, 无需调整')

            # --- 打印当前全部巡检点状态 ---
            self._dump_waypoint_state()

    # ========== 计划查询 ==========

    def _on_get_next_plan(self, req, resp):
        """返回按 (priority DESC, id ASC) 排序的巡检计划"""
        self.get_logger().info('[SCH][计划] 收到 get_next_plan 请求')
        try:
            plan = self.get_next_plan()
            resp.success = True
            resp.message = json.dumps(plan, ensure_ascii=False)

            self.get_logger().info(f'[SCH][计划] 返回 {len(plan)} 个巡检点:')
            top_half = plan[:max(3, len(plan) // 2)]
            for i, p in enumerate(top_half):
                self.get_logger().info(
                    f'[SCH][计划]   [{i + 1}] id={p["id"]} {p["name"]} '
                    f'priority={p["priority"]} dwell={p["dwell"]}s')
            if len(plan) > len(top_half):
                self.get_logger().info(
                    f'[SCH][计划]   ... 其余 {len(plan) - len(top_half)} 个点')

        except Exception as e:
            resp.success = False
            resp.message = f'计划生成失败: {e}'
            self.get_logger().error(f'[SCH][计划] 失败: {e}')
        return resp

    def get_next_plan(self) -> list[dict]:
        """
        返回排好序的巡检序列.
        排序: priority 降序 → id 升序
        """
        with self._lock:
            sorted_wps = sorted(self._waypoints, key=lambda w: (-w['priority'], w['id']))

            self.get_logger().info(
                f'[SCH][排序] 巡检计划排序完成 (priority DESC, id ASC):')
            for i, w in enumerate(sorted_wps):
                self.get_logger().info(
                    f'[SCH][排序]   [{i + 1}] id={w["id"]} {w["name"]} '
                    f'priority={w["priority"]} dwell={w["dwell"]}s')

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

    def _dump_waypoint_state(self):
        """打印当前全部巡检点快照"""
        lines = []
        for w in sorted(self._waypoints, key=lambda x: x['id']):
            lines.append(
                f'  id={w["id"]} {w["name"]}: '
                f'priority={w["priority"]} dwell={w["dwell"]}s '
                f'({w["lat"]:.4f}, {w["lng"]:.4f})')
        self.get_logger().info(
            f'[SCH][状态] 巡检点快照 ({len(self._waypoints)}):\n'
            + '\n'.join(lines))

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
