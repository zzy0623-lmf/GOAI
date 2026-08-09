"""
哨兵报告生成节点

  订阅 /sentinel/confirmed_alerts 持续收集告警.
  调用 ~/generate_report 服务触发报告生成, JSON 保存至 ~/sentinel_reports/.
"""

import rclpy
import json
import os
import uuid
import threading
from datetime import datetime, timezone
from pathlib import Path
from rclpy.node import Node
from std_srvs.srv import Trigger
from sentinel_interfaces.msg import AnomalyEvent


class ReportGenerator(Node):
    """巡检报告生成器"""

    REPORT_DIR = Path.home() / 'sentinel_reports'

    def __init__(self):
        super().__init__('report_generator')

        # ---- 告警收集 ----
        self._alerts: list[dict] = []
        self._lock = threading.Lock()
        self._mission_start = self.get_clock().now()

        self.create_subscription(
            AnomalyEvent, '/sentinel/confirmed_alerts', self._on_alert, 100
        )

        # ---- 报告生成服务 ----
        self._srv = self.create_service(
            Trigger, '~/generate_report', self._on_generate_report
        )

        # 确保目录存在
        self.REPORT_DIR.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(
            f'ReportGenerator 启动, 报告目录: {self.REPORT_DIR}')

    # ========== 告警收集 ==========

    def _on_alert(self, event: AnomalyEvent):
        alert_dict = {
            'event_id': event.event_id,
            'severity': self._severity_str(event.severity),
            'anomaly_type': event.anomaly_type,
            'confidence': round(event.confidence, 4),
            'description': event.description,
            'lat': event.lat,
            'lng': event.lng,
            'waypoint_id': event.waypoint_id,
            'detection_source': event.detection_source,
            'image_path': event.image_path,
            'received_at': self._now_iso(),
        }
        with self._lock:
            self._alerts.append(alert_dict)
        self.get_logger().info(
            f'收集告警 [{len(self._alerts)}] {event.anomaly_type} '
            f'conf={event.confidence:.2f}')

    # ========== 报告生成 ==========

    def _on_generate_report(self, req, resp):
        """服务回调: 生成并保存报告"""
        try:
            report = self._build_report()
            filepath = self._save_report(report)

            resp.success = True
            resp.message = f'报告已保存: {filepath}'
            self.get_logger().info(resp.message)

            # 重置收集器
            with self._lock:
                self._alerts.clear()
                self._mission_start = self.get_clock().now()

        except Exception as e:
            resp.success = False
            resp.message = f'报告生成失败: {e}'
            self.get_logger().error(resp.message)

        return resp

    def _build_report(self) -> dict:
        """构建报告数据"""
        now = self.get_clock().now()
        elapsed = (now - self._mission_start).nanoseconds / 1e9

        with self._lock:
            alerts = list(self._alerts)

        stats = self._compute_stats(alerts)

        report = {
            'report_id': f'rpt_{datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")}_{uuid.uuid4().hex[:8]}',
            'generated_at': self._now_iso(),
            'mission_duration_s': round(elapsed, 1),
            'total_alerts': stats['total'],
            'critical': stats['critical'],
            'warning': stats['warning'],
            'info': stats['info'],
            'alerts': alerts,
        }
        return report

    def _save_report(self, report: dict) -> str:
        """保存 JSON 到文件"""
        filename = f'{report["report_id"]}.json'
        filepath = self.REPORT_DIR / filename
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        return str(filepath)

    # ========== 统计 ==========

    @staticmethod
    def _compute_stats(alerts: list[dict]) -> dict:
        total = len(alerts)
        critical = sum(1 for a in alerts if a['severity'] == 'critical')
        warning  = sum(1 for a in alerts if a['severity'] == 'warning')
        info     = sum(1 for a in alerts if a['severity'] == 'info')
        return {
            'total': total,
            'critical': critical,
            'warning': warning,
            'info': info,
        }

    # ========== 工具 ==========

    @staticmethod
    def _severity_str(severity: int) -> str:
        return {0: 'info', 1: 'warning', 2: 'critical'}.get(severity, 'unknown')

    @staticmethod
    def _now_iso() -> str:
        return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def main(args=None):
    rclpy.init(args=args)
    node = ReportGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
