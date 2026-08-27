
from math import inf
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.qos import QoSProfile, DurabilityPolicy


from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OccMapping(Node):

    # --------------------------------------------------------------------------------------------------------
    def __init__(self):
        super().__init__('occ_mapping')
        # self.get_logger().info('OccMapping node started!')

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_call_back)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', qos)

        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
 

        # defining constants for log-odds of occupied and log-odds of free cells
        P_loc = 0.9
        P_free = 0.2

        self.l_occ = np.log(P_loc/(1-P_loc))
        self.l_free = np.log(P_free/(1-P_free))
        self.l_min = np.log(0.01/(1-0.01))
        self.l_max = np.log(0.99/(1-0.99))

        self.TRESHOLD_P_FREE = 0.3 
        self.TRESHOLD_P_OCC = 0.6

        self.once = True
        
        # map parameters
        self.map_width = 100
        self.map_height = 100
        self.resolution = 0.1
        self.origin_x = -5.0   # map origin x (world coord of cell (0,0))
        self.origin_y = -5.0   # map origin y (world coord of cell (0,0))

        # initialize occupancy grid as (rows, cols) = (height, width) and fill with l_min
        self.occupancy_grid = np.full((self.map_height, self.map_width), self.l_min, dtype=float)
        

    # --------------------------------------------------------------------------------------------------------
    def timer_call_back(self):
        if self.once:
            # print(f'\nlaser msg:{self.laser_msg.ranges}')
            # print(f'\nodom msg:{self.odom_msg.pose.pose.position}')
            self.once = False



    # --------------------------------------------------------------------------------------------------------
    def odom_callback(self, msg:Odometry):
        self.odom_msg = msg

        

    # --------------------------------------------------------------------------------------------------------
    def laser_callback(self, msg:LaserScan):
        self.laser_msg = msg
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid = np.isfinite(ranges)
        valid_ranges = ranges[valid]
        valid_angles = angles[valid]
        q = [self.odom_msg.pose.pose.orientation.x,
             self.odom_msg.pose.pose.orientation.y,
             self.odom_msg.pose.pose.orientation.z,
             self.odom_msg.pose.pose.orientation.w]
        _,_,yaw = euler_from_quaternion(q)
        # positions of the hit points in the laser frame

        start = self.get_grid_indice(self.odom_msg.pose.pose.position.x , self.odom_msg.pose.pose.position.y)
        x = valid_ranges*np.cos(valid_angles)
        y = valid_ranges*np.sin(valid_angles)
        x_hit_odom = x*np.cos(yaw) - y*np.sin(yaw)
        y_hit_odom = x*np.sin(yaw) + y*np.cos(yaw)

        x_hit_odom += self.odom_msg.pose.pose.position.x
        y_hit_odom += self.odom_msg.pose.pose.position.y



        for (x,y) in zip(x_hit_odom, y_hit_odom):
            x_cell, y_cell = self.get_grid_indice(x, y)

            
            end = (x_cell, y_cell) 
            line_cells = self.bresenham(start, end)
            for i, cell in enumerate(line_cells):
                if(i==len(line_cells)-1):
                    self.occupancy_grid[cell[0], cell[1]] += self.l_occ
                else:
                    self.occupancy_grid[cell[0], cell[1]] += self.l_free

                # capping the log-odds values to avoid them growing without bound when revisiting the same cells many times
                self.occupancy_grid[cell[0], cell[1]] = min(max(self.occupancy_grid[cell[0], cell[1]], self.l_min), self.l_max)

        self.publish_map()
    # --------------------------------------------------------------------------------------------------------
    def get_grid_indice(self, x, y, resolution=0.1):
        x_cell = int ( (x+5)/resolution )
        y_cell = int ( (y+5)/resolution )

        x_cell = min( max(x_cell, 0) , 99)
        y_cell = min( max(y_cell, 0) , 99)
        return x_cell, y_cell
    

    # def get_grid_indice(self, x, y, resolution=None):
    #     if resolution is None:
    #         resolution = self.resolution

    #     # column from x, row from y (using origin)
    #     col = int((x - self.origin_x) / resolution)
    #     row = int((y - self.origin_y) / resolution)

    #     # clamp to valid range
    #     col = min(max(col, 0), self.map_width - 1)
    #     row = min(max(row, 0), self.map_height - 1)
    #     return row, col


    # --------------------------------------------------------------------------------------------------------
    def bresenham(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dx = x1 - x0
        dy = y1 - y0
        line_cells = []
        step = max(abs(dx), abs(dy))
        if (step!=0):
            stepX = dx / step
            stepY = dy / step
            for i in range(step+1):
                line_cell = (  int(x0 + i*stepX) , int(y0 + i*stepY))
                line_cells.append(line_cell)

        return line_cells        


    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = 'map'
        msg.header.frame_id = 'odom'

        
        msg.info.resolution = 0.1
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -5.0




        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # converting log-odds to probabilities
        p = 1 / (1 + np.exp(-self.occupancy_grid))
        mask_occ = p > self.TRESHOLD_P_OCC
        mask_free = p < self.TRESHOLD_P_FREE
        mask_unknown = ~(mask_free | mask_occ)


        occ_grid = -1 * np.ones_like(self.occupancy_grid, dtype=int)
        occ_grid[mask_occ] = 100
        occ_grid[mask_free] = 0
        occ_grid[mask_unknown] = -1

        print("\nProbabilities min/max:", np.min(p), np.max(p))
        print("Occupied:", np.sum(occ_grid==100), "Free:", np.sum(occ_grid==0), "Unknown:", np.sum(occ_grid==-1))       
        # assigning the flattened map to the data field of the grid map
        msg.data = occ_grid.flatten().astype(np.int8).tolist()
        print("Publishing map with", len(msg.data), "cells")

        self.map_publisher.publish(msg)






def main():
    rclpy.init()
    node = OccMapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()