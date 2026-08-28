import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

from robile_interfaces.msg import ObjectDetection
from robile_interfaces.msg import ObjectDetectionArray


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('/home/ros2_ws/yolo11n.pt')

        # Subscribe to RGB camera
        self.subscription = self.create_subscription(
            Image,
            '/camera_front/image_raw',
            self.image_callback,
            10
        )

        # Publish YOLO detections
        self.detection_publisher = self.create_publisher(
            ObjectDetectionArray,
            '/yolo/detections',
            10
        )

        self.frame_count = 0

        self.get_logger().info('YOLO node started')
        self.get_logger().info('Model: YOLO11n')
        self.get_logger().info('Device: CUDA')
        self.get_logger().info('Publishing: /yolo/detections')

    def image_callback(self, msg):
        self.frame_count += 1

        # Process every 5th frame
        if self.frame_count % 5 != 0:
            return

        image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        results = self.model.predict(
            source=image,
            device=0,
            verbose=False
        )

        result = results[0]
        detections = result.boxes

        # Create detection array message
        detection_array = ObjectDetectionArray()

        # Preserve camera timestamp and frame
        detection_array.header = msg.header

        for box in detections:
            detection = ObjectDetection()

            detection.class_name = self.model.names[int(box.cls[0])]
            detection.confidence = float(box.conf[0])

            x1, y1, x2, y2 = box.xyxy[0].tolist()

            detection.x1 = float(x1)
            detection.y1 = float(y1)
            detection.x2 = float(x2)
            detection.y2 = float(y2)

            detection_array.detections.append(detection)

        # Publish all detections from this image
        self.detection_publisher.publish(detection_array)

        self.get_logger().info(
            f'Frame {self.frame_count}: '
            f'{len(detections)} objects detected'
        )

        for detection in detection_array.detections:
            self.get_logger().info(
                f'  {detection.class_name}: '
                f'{detection.confidence:.2f} '
                f'bbox=('
                f'{detection.x1:.0f},'
                f'{detection.y1:.0f},'
                f'{detection.x2:.0f},'
                f'{detection.y2:.0f})'
            )


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
