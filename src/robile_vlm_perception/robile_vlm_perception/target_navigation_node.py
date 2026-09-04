#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class TargetNavigationNode(Node):

    def __init__(self):
        super().__init__('target_navigation_node')

        # Parameters
        # Declare all configurable settings for topics, frames, and safety distance
        self.declare_parameter('target_topic', '/vlm/target_point')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('standoff_distance', 0.9)

        # Retrieve parameter values into instance variables for easy access
        self.target_topic = self.get_parameter(
            'target_topic').value
        self.target_frame = self.get_parameter(
            'target_frame').value
        self.map_frame = self.get_parameter(
            'map_frame').value
        self.base_frame = self.get_parameter(
            'base_frame').value
        self.standoff_distance = self.get_parameter(
            'standoff_distance').value

        # TF2
        # Set up a transform buffer and listener to handle coordinate frame lookups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Nav2 action client
        # Connect to Nav2's NavigateToPose action server to send navigation commands
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # Target subscriber
        # Subscribe to incoming 3D points (e.g., from a Vision-Language Model perception node)
        self.target_sub = self.create_subscription(
            PointStamped,
            self.target_topic,
            self.target_callback,
            10
        )

        # State flag to prevent spamming new goals while the robot is already driving somewhere
        self.goal_active = False

        self.get_logger().info(
            'Target navigation bridge started'
        )

        self.get_logger().info(
            f'Target topic: {self.target_topic}'
        )

        self.get_logger().info(
            f'Target frame: {self.target_frame}'
        )                                                                                                                                                                                                                                 

        self.get_logger().info(
            f'Map frame: {self.map_frame}'
        )

        self.get_logger().info(
            f'Base frame: {self.base_frame}'
        )

        self.get_logger().info(
            f'Standoff distance: {self.standoff_distance:.2f} m'
        )

    def target_callback(self, msg):

        # Drop incoming goals if we are already busy navigating to one
        if self.goal_active:
            self.get_logger().warn(
                'Navigation goal already active; ignoring new target.'
            )
            return

        # Make sure the incoming point actually specifies which frame it belongs to
        if not msg.header.frame_id:
            self.get_logger().warn(
                'Received target without frame_id.'
            )
            return

        self.get_logger().info(
            f'Received target: '
            f'frame={msg.header.frame_id}, '
            f'x={msg.point.x:.3f}, '
            f'y={msg.point.y:.3f}, '
            f'z={msg.point.z:.3f}'
        )

        # ---------------------------------------------------------
        # 1. Transform target point: odom -> map
        # ---------------------------------------------------------

        try:
            # Transform the target point into the global map frame so we have a common reference
            target_map = self.tf_buffer.transform(
                msg,
                self.map_frame,
                timeout=Duration(seconds=1.0)
            )

        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform target '
                f'{msg.header.frame_id} -> {self.map_frame}: {ex}'
            )
            return

        target_x = target_map.point.x
        target_y = target_map.point.y

        self.get_logger().info(
            f'Target in map: '
            f'x={target_x:.3f}, '
            f'y={target_y:.3f}, '
            f'z={target_map.point.z:.3f}'
        )

        # ---------------------------------------------------------
        # 2. Get current robot pose in map
        # ---------------------------------------------------------

        try:
            # Look up where the robot base currently is relative to the map frame
            robot_tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=1.0)
            )

        except TransformException as ex:
            self.get_logger().error(
                f'Could not get '
                f'{self.map_frame} -> {self.base_frame}: {ex}'
            )
            return

        robot_x = robot_tf.transform.translation.x
        robot_y = robot_tf.transform.translation.y

        self.get_logger().info(
            f'Robot in map: '
            f'x={robot_x:.3f}, '
            f'y={robot_y:.3f}'
        )

        # ---------------------------------------------------------
        # 3. Calculate direction robot -> object
        # ---------------------------------------------------------

        dx = target_x - robot_x
        dy = target_y - robot_y

        # Figure out how far away the target point is from our current position
        distance = math.hypot(dx, dy)

        self.get_logger().info(
            f'Target distance: {distance:.3f} m'
        )

        # If we are already closer than our required standoff distance, stop here
        if distance <= self.standoff_distance:
            self.get_logger().warn(
                f'Target is only {distance:.3f} m away. '
                f'Cannot create a {self.standoff_distance:.3f} m '
                f'standoff goal.'
            )
            return

        # Normalize the vector to get the direction unit vector
        direction_x = dx / distance
        direction_y = dy / distance

        # ---------------------------------------------------------
        # 4. Create safe stopping position
        #
        # Object
        #    ^
        #    |
        # 0.9 m
        #    |
        # Goal
        #    |
        # Robot
        # ---------------------------------------------------------

        # Pull back from the target point by the standoff distance so we don't crash into it
        goal_x = (
            target_x
            - self.standoff_distance * direction_x
        )

        goal_y = (
            target_y
            - self.standoff_distance * direction_y
        )

        # Calculate the orientation (yaw) so the robot faces directly towards the object
        goal_yaw = math.atan2(
            dy,
            dx
        )

        self.get_logger().info(
            f'Safe navigation goal: '
            f'x={goal_x:.3f}, '
            f'y={goal_y:.3f}, '
            f'yaw={goal_yaw:.3f} rad'
        )

        # ---------------------------------------------------------
        # 5. Build Nav2 PoseStamped
        # ---------------------------------------------------------

        goal_pose = PoseStamped()

        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Set the target coordinates for the navigation goal
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0

        # Convert the heading angle (yaw) into a quaternion for the orientation message
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(
            goal_yaw / 2.0
        )
        goal_pose.pose.orientation.w = math.cos(
            goal_yaw / 2.0
        )

        # ---------------------------------------------------------
        # 6. Send goal to Nav2
        # ---------------------------------------------------------

        # Make sure the Nav2 action server is actually online before trying to send anything
        if not self.nav_client.wait_for_server(
                timeout_sec=1.0):
            self.get_logger().error(
                'Nav2 /navigate_to_pose action server '
                'is not available.'
            )
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        # Lock out other incoming goals while this one runs
        self.goal_active = True

        self.get_logger().info(
            'Sending goal to /navigate_to_pose...'
        )

        # Send the goal asynchronously and assign a callback to track the response
        future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        try:
            goal_handle = future.result()
        except Exception as ex:
            self.goal_active = False

            self.get_logger().error(
                f'Failed to send Nav2 goal: {ex}'
            )
            return

        # Check if Nav2 accepted or rejected our navigation request
        if not goal_handle.accepted:
            self.goal_active = False

            self.get_logger().error(
                'Nav2 rejected the goal.'
            )
            return

        self.get_logger().info(
            'Nav2 accepted the goal.'
        )

        # Goal was accepted, now wait for the final execution result asynchronously
        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        # Continuously log how much distance is left on the current path if available
        if hasattr(feedback, 'distance_remaining'):
            self.get_logger().info(
                f'Nav2 distance remaining: '
                f'{feedback.distance_remaining:.2f} m'
            )

    def result_callback(self, future):

        # Free up our lock so we can accept new target points again
        self.goal_active = False

        try:
            result = future.result()

            status = result.status
            nav_result = result.result

        except Exception as ex:
            self.get_logger().error(
                f'Error receiving Nav2 result: {ex}'
            )
            return

        # Check if the robot successfully reached the safe standoff pose
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                '========================================'
            )
            self.get_logger().info(
                'NAVIGATION SUCCEEDED'
            )
            self.get_logger().info(
                'Robot reached the safe stopping pose.'
            )
            self.get_logger().info(
                '========================================'
            )

        else:
            self.get_logger().warn(
                f'Navigation finished with status: {status}'
            )

            # Print out any error codes or messages if Nav2 provides them
            if hasattr(nav_result, 'error_code'):
                self.get_logger().warn(
                    f'Nav2 error code: '
                    f'{nav_result.error_code}'
                )

            if hasattr(nav_result, 'error_msg'):
                self.get_logger().warn(
                    f'Nav2 error message: '
                    f'{nav_result.error_msg}'
                )


def main(args=None):

    # Initialize the ROS client library
    rclpy.init(args=args)

    # Instantiate our target navigation node
    node = TargetNavigationNode()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up gracefully when shutting down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()