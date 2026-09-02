import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge

from robile_interfaces.msg import VlmTarget

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class DepthTargetNode(Node):

    def __init__(self):
        super().__init__('depth_target_node')

        self.bridge = CvBridge()

        self.depth_buffer = {}
        self.max_buffer_size = 30

        self.latest_camera_info = None

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera_front/depth/image_raw',
            self.depth_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera_front/depth/camera_info',
            self.camera_info_callback,
            10
        )

        self.target_subscription = self.create_subscription(
            VlmTarget,
            '/vlm/target',
            self.target_callback,
            10
        )

        self.point_publisher = self.create_publisher(
            PointStamped,
            '/vlm/target_point',
            10
        )

        self.get_logger().info('Depth target node started')
        self.get_logger().info(
            'Subscribed to /camera_front/depth/image_raw'
        )
        self.get_logger().info(
            'Subscribed to /camera_front/depth/camera_info'
        )
        self.get_logger().info(
            'Subscribed to /vlm/target'
        )
        self.get_logger().info(
            'TF2 enabled: camera_front_link -> base_link'
        )
        self.get_logger().info(
            'Publishing 3D target on /vlm/target_point'
        )

    def depth_callback(self, msg):

        depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        timestamp = (
            msg.header.stamp.sec,
            msg.header.stamp.nanosec
        )

        self.depth_buffer[timestamp] = depth

        while len(self.depth_buffer) > self.max_buffer_size:
            oldest_timestamp = next(iter(self.depth_buffer))
            del self.depth_buffer[oldest_timestamp]

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def get_matching_depth(self, target_msg):

        timestamp = (
            target_msg.header.stamp.sec,
            target_msg.header.stamp.nanosec
        )

        if timestamp in self.depth_buffer:
            return self.depth_buffer[timestamp]

        if not self.depth_buffer:
            return None

        target_ns = (
            target_msg.header.stamp.sec * 1_000_000_000
            + target_msg.header.stamp.nanosec
        )

        closest_timestamp = min(
            self.depth_buffer.keys(),
            key=lambda t: abs(
                (
                    t[0] * 1_000_000_000
                    + t[1]
                ) - target_ns
            )
        )

        return self.depth_buffer[closest_timestamp]

    def target_callback(self, msg):

        if self.latest_camera_info is None:
            self.get_logger().warning(
                'No depth camera info available yet.'
            )
            return

        depth = self.get_matching_depth(msg)

        if depth is None:
            self.get_logger().warning(
                'No depth image matching the target timestamp.'
            )
            return

        height, width = depth.shape[:2]

        x1 = max(0, int(msg.x1))
        y1 = max(0, int(msg.y1))
        x2 = min(width - 1, int(msg.x2))
        y2 = min(height - 1, int(msg.y2))

        if x2 <= x1 or y2 <= y1:
            self.get_logger().warning(
                'Invalid target bounding box.'
            )
            return

        # Use an inner region of the bounding box to reduce
        # contamination from object boundaries/background.
        margin_x = max(1, int((x2 - x1) * 0.20))
        margin_y = max(1, int((y2 - y1) * 0.20))

        roi = depth[
            y1 + margin_y:y2 - margin_y + 1,
            x1 + margin_x:x2 - margin_x + 1
        ]

        valid_depth = roi[
            np.isfinite(roi) &
            (roi > 0.1) &
            (roi < 100.0)
        ]

        if valid_depth.size == 0:
            self.get_logger().warning(
                'No valid depth values inside target bounding box.'
            )
            return

        # Median is more robust than one center pixel.
        z = float(np.median(valid_depth))

        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0

        fx = self.latest_camera_info.k[0]
        fy = self.latest_camera_info.k[4]
        cx = self.latest_camera_info.k[2]
        cy = self.latest_camera_info.k[5]

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning(
                'Invalid camera intrinsics.'
            )
            return

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # -------------------------------------------------
        # 3D point in camera_front_link
        # -------------------------------------------------

        camera_point = PointStamped()

        camera_point.header.stamp = msg.header.stamp
        camera_point.header.frame_id = 'camera_front_link'

        camera_point.point.x = x
        camera_point.point.y = y
        camera_point.point.z = z

        # -------------------------------------------------
        # Transform camera point -> odom using TF2
        # -------------------------------------------------

        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'camera_front_link',
                camera_point.header.stamp,
                timeout=rclpy.duration.Duration(
                    seconds=0.5
                )
            )

            odom_point = do_transform_point(
                camera_point,
                transform
            )

            # do_transform_point() may use the TF transform
            # timestamp. Restore the original target timestamp.
            odom_point.header.stamp = camera_point.header.stamp
            odom_point.header.frame_id = 'odom'

        except Exception as e:
            self.get_logger().warning(
                f'Could not transform target point '
                f'from camera_front_link to odom: {e}'
            )
            return

        # Publish the transformed point.
        self.point_publisher.publish(odom_point)

        self.get_logger().info(
            f'Target #{msg.target_id} '
            f'({msg.target_class})'
        )

        self.get_logger().info(
            f'Camera frame: '
            f'x={x:.3f} m, '
            f'y={y:.3f} m, '
            f'z={z:.3f} m'
        )

        self.get_logger().info(
            f'Odom frame: '
            f'x={odom_point.point.x:.3f} m, '
            f'y={odom_point.point.y:.3f} m, '
            f'z={odom_point.point.z:.3f} m'
        )

        self.get_logger().info(
            'Published transformed target on /vlm/target_point'
        )


def main(args=None):

    rclpy.init(args=args)

    node = DepthTargetNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()