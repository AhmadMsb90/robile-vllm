#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robile_interfaces.msg import PositionLabelled, PositionLabelledArray
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_e


class LocalisationUsingKalmanFilter(Node):
    """
    Landmark based localisation using Kalman Filter
    This is a partially structured class for AMR assignment
    """

    def __init__(self):
        super().__init__('localisation_using_kalman_filter')

        # declaring and getting parameters from yaml file
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_frame', 'map'),
                ('odom_frame', 'odom'),                
                ('laser_link_frame', 'base_laser_front_link'),
                ('real_base_link_frame', 'real_base_link'),
                ('scan_topic', 'scan'),
                ('odom_topic', 'odom'),
                ('rfid_tag_poses_topic', 'rfid_tag_poses'),
                ('initial_pose_topic', 'initialpose'),
                ('real_base_link_pose_topic', 'real_base_link_pose'),
                ('estimated_base_link_pose_topic', 'estimated_base_link_pose'),
                ('minimum_travel_distance', 0.1),
                ('minimum_travel_heading', 0.1),
                ('rfid_tags.A', [1.,1.]),
                ('rfid_tags.B', [6.,1.]),
                ('rfid_tags.C', [3.,-1.]),
                ('rfid_tags.D', [1.,-3.]),
                ('rfid_tags.E', [4.,-4.]),                        
            ])
            

        
        self.covariance = np.diag([0.01, 0.01, 0.01])
        self.Q = np.diag([0.01, 0.01, 0.01])  # process noise for x, y, yaw

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.laser_link_frame = self.get_parameter('laser_link_frame').get_parameter_value().string_value
        self.real_base_link_frame = self.get_parameter('real_base_link_frame').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.rfid_tag_poses_topic = self.get_parameter('rfid_tag_poses_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value
        self.real_base_link_pose_topic = self.get_parameter('real_base_link_pose_topic').get_parameter_value().string_value
        self.estimated_base_link_pose_topic = self.get_parameter('estimated_base_link_pose_topic').get_parameter_value().string_value
        self.minimum_travel_distance = self.get_parameter('minimum_travel_distance').get_parameter_value().double_value
        self.minimum_travel_heading = self.get_parameter('minimum_travel_heading').get_parameter_value().double_value
        self.rfid_tags_A = self.get_parameter('rfid_tags.A').get_parameter_value().double_array_value
        self.rfid_tags_B = self.get_parameter('rfid_tags.B').get_parameter_value().double_array_value
        self.rfid_tags_C = self.get_parameter('rfid_tags.C').get_parameter_value().double_array_value
        self.rfid_tags_D = self.get_parameter('rfid_tags.D').get_parameter_value().double_array_value
        self.rfid_tags_E = self.get_parameter('rfid_tags.E').get_parameter_value().double_array_value


        self.tag_map = {
        'A': self.rfid_tags_A,
        'B': self.rfid_tags_B,
        'C': self.rfid_tags_C,
        'D': self.rfid_tags_D,
        'E': self.rfid_tags_E
                        }
        

        # to store odometry pose (2D) [x, y, yaw]
        self.predicted_state = np.zeros((3,1))
        self.odom_msg = Odometry()

        # setting up laser scan and rfid tag subscribers
        self.rfid_tag_subscriber = self.create_subscription(PositionLabelledArray, self.rfid_tag_poses_topic, self.rfid_callback, 10)
        self.real_laser_link_subscriber = self.create_subscription(PoseStamped, self.real_base_link_pose_topic, self.real_base_link_pose_callback, 10)        
        self.estimated_robot_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, self.estimated_base_link_pose_topic, 10)
        
        # odom subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # timer to publish estimated pose with covariance
        timer_period = 0.1
        self.timer = self.create_timer(0.1, self.timer_callback)


        # setting up tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def timer_callback(self):
        estimated_pose_msg = PoseWithCovarianceStamped()
        estimated_pose_msg.pose.pose.position.x = float(self.predicted_state[0])
        estimated_pose_msg.pose.pose.position.y = float(self.predicted_state[1])
        estimated_pose_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, float(self.predicted_state[2]))
        estimated_pose_msg.pose.pose.orientation.x = q[0]
        estimated_pose_msg.pose.pose.orientation.y = q[1]
        estimated_pose_msg.pose.pose.orientation.z = q[2]
        estimated_pose_msg.pose.pose.orientation.w = q[3]


        estimated_pose_msg.pose.covariance = (0.01*np.eye(6)).flatten().tolist()
        self.estimated_robot_pose_publisher.publish(estimated_pose_msg)




    def odom_callback(self, msg:Odometry):
        self.odom_msg = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _,_,yaw = euler_from_quaternion(orientation_list)
        self.predicted_state[0] = x
        self.predicted_state[1] = y
        self.predicted_state[2] = yaw
        # That’s a simplified version of the EKF prediction equation: P = F @ P @ F.T + Q  
        # Just take the F as I because we just take odom state as the new state
        self.covariance += self.Q   
        print(f"\n\nrobot's pose(x, y, yaw) = ({x:.2f},{y:.2f},{yaw:.2f})")

        
        

 
    def rfid_callback(self, msg:PositionLabelledArray):
        """
        Based on the detected RFID tags, performing measurement update
        """
        if len(msg.positions) == 0:
            self.get_logger().info("No RFID tags detected")
            return
        
        for tag in msg.positions:
            name = tag.name
            x_rel = tag.position.x
            y_rel = tag.position.y
            self.get_logger().info(f"Detected tag {name} at ({x_rel:.2f}, {y_rel:.2f})")


            (x_tag, y_tag) = self.tag_map[name]
            dx = x_tag - float(self.predicted_state[0])
            dy = y_tag - float(self.predicted_state[1])
            yaw = float(self.predicted_state[2])




            R = np.array([[np.cos(yaw), np.sin(yaw)],
                            [-np.sin(yaw), np.cos(yaw)]])
            
            h = R.T @ np.array([[dx], [dy]])
            z = np.array([x_rel, y_rel])
            y = z - h.flatten()
            d_R_t = np.array([[-np.sin(yaw), np.cos(yaw)],
                            [-np.cos(yaw), -np.sin(yaw)]])
            H = np.hstack((-R.T, d_R_t @ np.array([[dx], [dy]])))

            R_meas = np.diag([0.1, 0.1])
            S = H @ self.covariance @ H.T + R_meas
            K = self.covariance @ H.T @ np.linalg.inv(S)
            self.predicted_state += (K @ y.reshape(2,1))
            self.covariance = (np.eye(3) - K @ H) @ self.covariance
            self.predicted_state[2] = (self.predicted_state[2] + np.pi) % (2*np.pi) - np.pi


        ### YOUR CODE HERE ###
        
        return

    def real_base_link_pose_callback(self, msg):
        """
        Updating the base_link pose based on the update in robile_rfid_tag_finder.py
        """
        yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.real_laser_link_pose = [msg.pose.position.x, msg.pose.position.y, yaw]


def main(args=None):
    rclpy.init(args=args)

    localisation_using_kalman_filter = None  # <-- prevent UnboundLocalError
    try:
        localisation_using_kalman_filter = LocalisationUsingKalmanFilter()
        rclpy.spin(localisation_using_kalman_filter)

    finally:
        if localisation_using_kalman_filter is not None:
            localisation_using_kalman_filter.destroy_node()
        if rclpy.ok():  # ensures shutdown only happens once
            rclpy.shutdown()


if __name__ == '__main__':
    main()
