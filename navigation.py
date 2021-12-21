import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from .rviz import RvizInterface
import numpy as np
import math
import cmath
import time
from math import atan2
import matplotlib.pyplot as plt
import scipy.stats

scanfile = 'lidar.txt'
mapfile = 'map.txt'
occ_bins = [-1, 0, 60, 100]

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class Robot(Node):
    def __init__(self):
        super().__init__('navigation')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

            # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning

        self.string_subscription = self.create_subscription(
            String,
            'balldetect',
            self.string_callback,
            10)

        self.laser_range = np.array([])

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.balldetect = False
        self.balldirection = 'forward'

    def string_callback(self,msg):
        self.get_logger().info('In shooter callback %s' % msg.data)
        if msg.data == 'Stop moving':
            self.balldetect = True
            self.get_logger().info('Stopping because ball is detected')
        elif msg.data == 'Continue moving':
            self.balldetect = False
            self.get_logger().info('Resuming navigation ...')
        elif msg.data == 'left':
            self.balldirection = 'left'
            self.get_logger().info('going left')
        elif msg.data == 'right':
            self.balldirection = 'right'
            self.get_logger().info('going right')
        elif msg.data == 'forward':
            self.balldirection = 'forward'
            self.get_logger().info('going forward')
        elif msg.data == 'stop':
            self.balldirection = 'stop'
            self.get_logger().info('stopping')

    def odom_callback(self, msg):
        #self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
    
    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
    
    def occ_callback(self, msg):
        self.get_logger().info('In occ_callback')
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        iwidth = msg.info.width
        iheight = msg.info.height
        
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        
        # reshape to 2D array using column order
        self.occdata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
                    
        #get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
    
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time(), rclpy.time.Duration())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(e)
            self.get_logger().info('No transformation found')
            self.stopbot()
            return
                
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation

        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)

        self.cur_rot = yaw
        
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))

        self.x = grid_x
        self.y = grid_y

        # set current robot location to 0
        self.occdata[grid_y][grid_x] = 0

        self.get_logger().info("Map size is %i %i " % (len(self.occdata), len(self.occdata[0])))
        self.get_logger().info("Position now is %i %i " % (grid_x,grid_y))

        # transposed = self.occdata.
        # transposed = transposed.transpose()
        # transposed = np.rot90(transposed, 1)

        if(self.occdata.any()):
            visualization = RvizInterface(self.occdata, msg)

        if not(self.path):
            self.get_logger().info('Empty path')
            return
        else:
            # for (i,j) in self.path:
            #     transposed[len(transposed) - j][i] = 0
            self.get_logger().info("Hey path is published")
            visualization.publishPath(self.path)
        
        # np.savetxt(mapfile, transposed, fmt='%d', delimiter='')
        np.savetxt('occ.txt', self.occdata, fmt='%d', delimiter='')

    def stop(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def move(self):
        try:
            rclpy.spin_once(self)
            
            while rclpy.ok():
                # Ball is detected, stop navigation, and send twist commands back to RPi and RPi to STM32
                if self.balldetect:
                    twist = Twist()
                    if self.shooterDirection == 'f':
                        self.get_logger().info(" forward in mover")
                        twist.linear.x = -0.1
                        twist.angular.z = 0.0
                    elif self.shooterDirection == 'l':
                        self.get_logger().info("left in mover")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.1
                    elif self.shooterDirection == 'r':   
                        self.get_logger().info(" right in mover")
                        twist.linear.x = 0.0
                        twist.angular.z = -0.1
                    elif self.shooterDirection == 's':
                        self.get_logger().info(" stopping in mover")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    self.publisher_.publish(twist)                     
                    rclpy.spin_once(self)
                else:
                    twist = Twist()

        except Exception as e:
            print(e, "Exception")
        finally:
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    robot.move()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()