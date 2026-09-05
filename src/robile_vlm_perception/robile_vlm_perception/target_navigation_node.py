#!/usr/bin/env python3

# Import standard Python math library for geometric calculations (hypot, atan2, sin, cos)
import math

# Import core ROS 2 client library for Python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

# Import ROS 2 standard message types for spatial points and poses
from geometry_msgs.msg import PointStamped, PoseStamped
# Import Nav2 action definition for sending navigation goal requests
from nav2_msgs.action import NavigateToPose
# Import action status constants to check goal execution outcomes
from action_msgs.msg import GoalStatus

# Import TF2 components for handling coordinate frame lookups and spatial transforms
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


# Define the main ROS 2 node class inheriting from rclpy.node.Node
class TargetNavigationNode(Node):

    def __init__(self):
        # Initialize the parent Node class with a unique internal node name
        super().__init__('target_navigation_node')

        # Parameters
        # Declare all configurable settings for topics, frames, and safety distance with default values
        self.declare_parameter('target_topic', '/vlm/target_point')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('standoff_distance', 0.9)

        # Retrieve parameter values into instance variables for easy access throughout the node lifecycle
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
        # Set up a transform buffer to cache transform history and a listener to ingest tf broadcasts from the network
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Nav2 action client
        # Connect to Nav2's NavigateToPose action server to send autonomous navigation commands
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # Target subscriber
        # Subscribe to incoming 3D points (e.g., from a Vision-Language Model perception node) with a queue size of 10
        self.target_sub = self.create_subscription(
            PointStamped,
            self.target_topic,
            self.target_callback,
            10
        )

        # State flag to prevent spamming new goals while the robot is already driving somewhere
        self.goal_active = False

        # Log initialization message indicating node startup
        self.get_logger().info(
            'Target navigation bridge started'
        )

        # Log the configured target topic name
        self.get_logger().info(
            f'Target topic: {self.target_topic}'
        )

        # Log the configured target coordinate frame ID
        self.get_logger().info(
            f'Target frame: {self.target_frame}'
        )                                                                                                                                                                                                                                                                                            

        # Log the configured global map frame ID
        self.get_logger().info(
            f'Map frame: {self.map_frame}'
        )

        # Log the configured robot base frame ID
        self.get_logger().info(
            f'Base frame: {self.base_frame}'
        )

        # Log the configured standoff stopping distance with formatting precision
        self.get_logger().info(
            f'Standoff distance: {self.standoff_distance:.2f} m'
        )

    # Callback executed automatically whenever a new PointStamped message arrives on the target topic
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

        # Log detailed info about the incoming raw target point coordinates and frame
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
            # Transform the target point into the global map frame so we have a common reference frame
            target_map = self.tf_buffer.transform(
                msg,
                self.map_frame,
                timeout=Duration(seconds=1.0)
            )

        # Handle potential lookup exceptions if the transform tree is disconnected or lagging
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform target '
                f'{msg.header.frame_id} -> {self.map_frame}: {ex}'
            )
            return

        # Extract 2D coordinates from the transformed map point message
        target_x = target_map.point.x
        target_y = target_map.point.y

        # Log the newly converted coordinates of the target in the global map frame
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
            # Look up where the robot base currently is relative to the map frame using latest available transform (Time())
            robot_tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=1.0)
            )

        # Catch transform errors if base_frame cannot be located relative to map_frame
        except TransformException as ex:
            self.get_logger().error(
                f'Could not get '
                f'{self.map_frame} -> {self.base_frame}: {ex}'
            )
            return

        # Extract current translation values for robot x and y position in the map
        robot_x = robot_tf.transform.translation.x
        robot_y = robot_tf.transform.translation.y

        # Log the robot's current estimated position in the map frame
        self.get_logger().info(
            f'Robot in map: '
            f'x={robot_x:.3f}, '
            f'y={robot_y:.3f}'
        )

        # ---------------------------------------------------------
        # 3. Calculate direction robot -> object
        # ---------------------------------------------------------

        # Compute delta components along x and y axes between robot and target
        dx = target_x - robot_x
        dy = target_y - robot_y

        # Figure out how far away the target point is from our current position using Euclidean distance formula
        distance = math.hypot(dx, dy)

        # Log the calculated absolute distance to the target
        self.get_logger().info(
            f'Target distance: {distance:.3f} m'
        )

        # If we are already closer than our required standoff distance, stop here to avoid reverse movement issues
        if distance <= self.standoff_distance:
            self.get_logger().warn(
                f'Target is only {distance:.3f} m away. '
                f'Cannot create a {self.standoff_distance:.3f} m '
                f'standoff goal.'
            )
            return

        # Normalize the vector to get the direction unit vector components
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

        # Pull back from the target point along the directional vector by the standoff distance so we don't crash into it
        goal_x = (
            target_x
            - self.standoff_distance * direction_x
        )

        goal_y = (
            target_y
            - self.standoff_distance * direction_y
        )

        # Calculate the orientation (yaw angle) using atan2 so the robot faces directly towards the object
        goal_yaw = math.atan2(
            dy,
            dx
        )

        # Log the calculated intermediate goal pose components (x, y, and yaw)
        self.get_logger().info(
            f'Safe navigation goal: '
            f'x={goal_x:.3f}, '
            f'y={goal_y:.3f}, '
            f'yaw={goal_yaw:.3f} rad'
        )

        # ---------------------------------------------------------
        # 5. Build Nav2 PoseStamped
        # ---------------------------------------------------------

        # Instantiate a new PoseStamped message container for Nav2
        goal_pose = PoseStamped()

        # Assign the map frame ID and current ROS timestamp to the message header
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Set the target coordinates for the navigation goal position
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0

        # Convert the heading angle (yaw) into standard quaternion components (roll=0, pitch=0) for orientation message
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

        # Make sure the Nav2 action server is actually online and responsive before trying to send anything
        if not self.nav_client.wait_for_server(
                timeout_sec=1.0):
            self.get_logger().error(
                'Nav2 /navigate_to_pose action server '
                'is not available.'
            )
            return

        # Create an instance of NavigateToPose.Goal and populate it with our goal pose
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        # Lock out other incoming goals while this one runs by setting the active flag to True
        self.goal_active = True

        # Log intent to send the goal request to the action server
        self.get_logger().info(
            'Sending goal to /navigate_to_pose...'
        )

        # Send the goal asynchronously to Nav2 and assign a feedback callback to track live progress updates
        future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.feedback_callback
        )

        # Register a callback function to handle the action server's initial acceptance/rejection response
        future.add_done_callback(
            self.goal_response_callback
        )

    # Callback executed when the action server responds to whether it accepted or rejected the goal request
    def goal_response_callback(self, future):

        try:
            # Retrieve the goal handle object from the completed future
            goal_handle = future.result()
        except Exception as ex:
            # Reset active flag if an exception occurred while fetching the goal response
            self.goal_active = False

            self.get_logger().error(
                f'Failed to send Nav2 goal: {ex}'
            )
            return

        # Check if Nav2 accepted or rejected our navigation request
        if not goal_handle.accepted:
            # Free up the lock flag since the goal was refused
            self.goal_active = False

            self.get_logger().error(
                'Nav2 rejected the goal.'
            )
            return

        # Log confirmation that the server has accepted the navigation request
        self.get_logger().info(
            'Nav2 accepted the goal.'
        )

        # Goal was accepted, now wait for the final execution result asynchronously via a new future
        result_future = (
            goal_handle.get_result_async()
        )

        # Register a callback function to process the final status once navigation completes
        result_future.add_done_callback(
            self.result_callback
        )

    # Callback executed continuously by Nav2 to provide intermediate execution status/feedback
    def feedback_callback(self, feedback_msg):

        # Extract the feedback structure from the message wrapper
        feedback = feedback_msg.feedback

        # Continuously log how much distance is left on the current path if the property exists
        if hasattr(feedback, 'distance_remaining'):
            self.get_logger().info(
                f'Nav2 distance remaining: '
                f'{feedback.distance_remaining:.2f} m'
            )

    # Callback executed once the action goal finishes execution (whether success, failure, or cancellation)
    def result_callback(self, future):

        # Free up our lock flag so we can accept new target points again
        self.goal_active = False

        try:
            # Extract result payload objects from the completed future object
            result = future.result()

            status = result.status
            nav_result = result.result

        except Exception as ex:
            self.get_logger().error(
                f'Error receiving Nav2 result: {ex}'
            )
            return

        # Check if the robot successfully reached the safe standoff pose based on status constants
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
            # Handle non-successful termination statuses (aborted, canceled, etc.)
            self.get_logger().warn(
                f'Navigation finished with status: {status}'
            )

            # Print out any error codes if Nav2 provides them in the result message
            if hasattr(nav_result, 'error_code'):
                self.get_logger().warn(
                    f'Nav2 error code: '
                    f'{nav_result.error_code}'
                )

            # Print out any error text descriptions if Nav2 provides them
            if hasattr(nav_result, 'error_msg'):
                self.get_logger().warn(
                    f'Nav2 error message: '
                    f'{nav_result.error_msg}'
                )


# Entry point function for executing the ROS 2 node script
def main(args=None):

    # Initialize the ROS client library runtime environment
    rclpy.init(args=args)

    # Instantiate our target navigation node class
    node = TargetNavigationNode()

    try:
        # Keep the node running, listening for subscriptions and managing callbacks until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gracefully catch manual termination signals like Ctrl+C
        pass

    # Clean up node resources and shut down the ROS client context gracefully
    node.destroy_node()
    rclpy.shutdown()


# Standard Python entry check block to execute main() when run directly as a script
if __name__ == '__main__':
    main()