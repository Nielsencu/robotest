import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import serial

import numpy as np
class Comms(Node):
    def __init__(self):
        # Create publisher to send odometry data, laser scan data, and occupancy grid data
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.occ_publisher = self.create_publisher(OccupancyGrid, 'map', qos_profile_sensor_data)

        # Subscribe to twist message to send to STM32
        self.twist_subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

        # Variable initialization
        self.roll,self.pitch,self.yaw = 0,0,0
        self.laser_range = np.array([])

    def twist_callback(self,msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z # should be 0

        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z

        ser = serial.Serial()
        ser.port = '/dev/ttyAMA0'
        ser.baudrate = 115200
        ser.timeout = 60  # 1 min
        ser.open()

        msg = ''
        while True:
            char = ser.read(1)  # 1 byte
            msg = msg+char.decode('utf-8')
            if char == b'\r':
                break

        # Send pwm values here based on speed commands through serial com to STM32

    
if __name__ == '__main__':
    comms = Comms()
    twist_msg = Twist()
    twist_msg.linear.x = 1.0
    twist_msg.linear.y = 1.0
    twist_msg.linear.z = 1.0

    comms.twist_callback(twist_msg)
