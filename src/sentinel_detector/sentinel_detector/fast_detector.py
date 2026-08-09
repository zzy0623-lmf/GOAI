"""
哨兵快速检测节点 (YOLOv8)

  面向山猫 S10 四向全向相机系统.
  支持多路 RGB 同时检测, conf>0.85 直接告警, 0.50~0.85 转 VLM 精判.
  输出标注图像到 /sentinel/annotated_image, 告警到 /sentinel/fast_alerts.

  S10 硬件配置: 4x 超广角相机 + 128线 LiDAR + IMU
  URL: https://www.deeprobotics.cn/robot/lynxs10.html
"""

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
    """YOLO 快速检测节点 — 多摄像头版"""

    YOLO_CONF_HIGH = 0.85    # 高于此值直接告警
    YOLO_CONF_LOW  = 0.50    # 低于此值忽略, 介于之间转 VLM

    # 山猫 S10 四向相机 topics
    CAMERA_TOPICS = [
        '/sensor/camera/front/image_raw',
        '/sensor/camera/left/image_raw',
        '/sensor/camera/right/image_raw',
        '/sensor/camera/rear/image_raw',
    ]
    # 向后兼容: 也支持单相机 /sensor/camera/rgb
    LEGACY_TOPICS = ['/sensor/camera/rgb']

    def __init__(self):
        super().__init__('fast_detector')

        # --- 参数 ---
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('conf_high', 0.85)
        self.declare_parameter('conf_low', 0.50)
        self.declare_parameter('multi_camera', True)

        model_name = self.get_parameter('model_name').value
        self.YOLO_CONF_HIGH = self.get_parameter('conf_high').value
        self.YOLO_CONF_LOW = self.get_parameter('conf_low').value
        use_multi = self.get_parameter('multi_camera').value

        # --- 模型 ---
        self.get_logger().info(f'加载 YOLO 模型: {model_name} ...')
        self.model = YOLO(model_name)
        self.bridge = CvBridge()

        # --- 订阅多路摄像头 ---
        self._subs = []
        if use_multi:
            self.get_logger().info(f'多摄像头模式: {len(self.CAMERA_TOPICS)} 路')
            for topic in self.CAMERA_TOPICS:
                sub = self.create_subscription(
                    Image, topic, self._make_callback(topic), 10
                )
                self._subs.append(sub)
        else:
            # 向后兼容: 单相机
            for topic in self.LEGACY_TOPICS:
                sub = self.create_subscription(
                    Image, topic, self._make_callback(topic), 10
                )
                self._subs.append(sub)

        # --- 发布 ---
        self.alert_pub = self.create_publisher(
            AnomalyEvent, '/sentinel/fast_alerts', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/sentinel/annotated_image', 10
        )

        # 速率控制: 避免过频检测 (S10 最高 30fps × 4 路)
        self._last_detect_time = {}
        self.declare_parameter('detect_interval', 0.5)  # 每路相机 0.5s 间隔
        self._detect_interval = self.get_parameter('detect_interval').value

        self.get_logger().info('FastDetector 启动完成 (山猫 S10 适配)')

    def _make_callback(self, topic_name: str):
        """为每个相机 topic 生成独立回调"""
        camera_id = topic_name.rstrip('/image_raw').replace('/sensor/camera/', '')

        def callback(msg: Image):
            # 频率控制
            now = self.get_clock().now().nanoseconds / 1e9
            last = self._last_detect_time.get(camera_id, 0.0)
            if now - last < self._detect_interval:
                return
            self._last_detect_time[camera_id] = now

            self._image_callback(msg, camera_id)
        return callback

    def _image_callback(self, msg: Image, camera_id: str = 'rgb'):
        """图像回调: YOLO 检测 → 分级告警 + 画框发布"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'[{camera_id}] cv_bridge 转换失败: {e}')
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

                self.publish_alert(conf, cls_name, severity, msg, camera_id)
                alert_issued = True

                # --- 画框 ---
                color = (0, 0, 255) if severity == AnomalyEvent.WARNING else (0, 255, 255)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f'{cls_name} {conf:.2f}'
                cv2.putText(annotated, label, (x1, max(y1 - 8, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # --- 发布标注图 ---
        if alert_issued:
            # 叠加相机来源信息
            cv2.putText(annotated, f'S10 cam: {camera_id}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            anno_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            anno_msg.header = msg.header
            self.annotated_pub.publish(anno_msg)

    def publish_alert(self, conf: float, cls_name: str, severity: int,
                      img_msg: Image, camera_id: str = 'rgb'):
        event = AnomalyEvent()
        event.event_id = f'yolo_{uuid.uuid4().hex[:12]}'
        event.severity = severity
        event.anomaly_type = cls_name
        event.confidence = conf
        event.description = f'YOLO({camera_id}) 检测到 {cls_name} (conf={conf:.2f})'
        event.lat = 0.0
        event.lng = 0.0
        event.waypoint_id = 0
        event.detection_source = 'yolo'
        event.image_path = ''

        self.alert_pub.publish(event)
        level = 'WARNING' if severity == AnomalyEvent.WARNING else 'INFO'
        self.get_logger().info(
            f'[{level}] [{camera_id}] {cls_name} conf={conf:.2f} → id={event.event_id}')


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
