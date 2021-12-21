import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import time

# Node that publishes ball detection message to ROS 
class BallDetector(Node):
    def __init__(self,msgdata):
        super().__init__('balldetector')
        self.publisher_ = self.create_publisher(String, 'balldetector', 10)
        # Publishing message at 5 Hz
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msgdata = msgdata

    def timer_callback(self):
        msg = String()
        msg.data = self.msgdata
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    ballDetector = BallDetector("Stop moving")
    rclpy.spin_once(ballDetector)
    time.sleep(2)
    ballDetector.msgdata = "Continue moving"
    rclpy.spin_once(ballDetector)
    ballDetector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()