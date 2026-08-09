import rclpy
import cv2
import uuid
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from sentinel_interfaces.msg import AnomalyEvent


class FastDetector(Node):
    """YOLO 快速检测节点"""

    YOLO_CONF_HIGH = 0.85    # 高于此值直接告警
    YOLO_CONF_LOW  = 0.50    # 低于此值忽略, 介于之间转 VLM

    def __init__(self):
        super().__init__('fast_detector')

        # --- 模型 ---
        self.get_logger().info('加载 YOLO 模型: yolov8n.pt ...')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()

        # --- 订阅 ---
        self.sub = self.create_subscription(
            Image, '/sensor/camera/rgb', self.image_callback, 10
        )

        # --- 发布 ---
        self.alert_pub = self.create_publisher(
            AnomalyEvent, '/sentinel/fast_alerts', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/sentinel/annotated_image', 10
        )

        self.get_logger().info('FastDetector 启动完成')

    def image_callback(self, msg: Image):
        """图像回调: YOLO 检测 → 分级告警 + 画框发布"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 转换失败: {e}')
            return

        results = self.model(frame, verbose=False)

        annotated = frame.copy()
        alert_issued = False

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < self.YOLO_CONF_LOW:
                    continue

                cls_id = int(box.cls[0])
                cls_name = self.model.names.get(cls_id, f'class_{cls_id}')
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # --- 分级告警 ---
                if conf > self.YOLO_CONF_HIGH:
                    severity = AnomalyEvent.WARNING
                else:
                    severity = AnomalyEvent.INFO

                self.publish_alert(conf, cls_name, severity, msg)
                alert_issued = True

                # --- 画框 ---
                color = (0, 0, 255) if severity == AnomalyEvent.WARNING else (0, 255, 255)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f'{cls_name} {conf:.2f}'
                cv2.putText(annotated, label, (x1, max(y1 - 8, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # --- 发布标注图 ---
        if alert_issued:
            anno_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            anno_msg.header = msg.header
            self.annotated_pub.publish(anno_msg)

    def publish_alert(self, conf: float, cls_name: str, severity: int, img_msg: Image):
        event = AnomalyEvent()
        event.event_id = f'yolo_{uuid.uuid4().hex[:12]}'
        event.severity = severity
        event.anomaly_type = cls_name
        event.confidence = conf
        event.description = f'YOLO 检测到 {cls_name} (conf={conf:.2f})'
        event.lat = 0.0
        event.lng = 0.0
        event.waypoint_id = 0
        event.detection_source = 'yolo'
        event.image_path = ''

        self.alert_pub.publish(event)
        level = 'WARNING' if severity == AnomalyEvent.WARNING else 'INFO'
        self.get_logger().info(f'[{level}] {cls_name} conf={conf:.2f} → id={event.event_id}')


def main(args=None):
    rclpy.init(args=args)
    node = FastDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
