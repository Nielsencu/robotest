from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from math import floor
from rclpy.node import Node


class RvizInterface(Node):
    def __init__(self,map, msg):
        super().__init__('rviz_interface')
        # self.pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=1,
        #                                latch=True)
        
        self.pub_path = self.create_publisher(Path,'global_plan',1)
    
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.resolution = msg.info.resolution
        # self.map.info.width = int(cfg.MAP_WIDTH / cfg.RESOLUTION)
        # self.map.info.height = int(cfg.MAP_HEIGHT / cfg.RESOLUTION)
        self.map.info.origin.position.x = msg.info.origin.position.x
        self.map.info.origin.position.y = msg.info.origin.position.y
        self.map.info.origin.position.z = msg.info.origin.position.z
    
        self.path = Path()
        self.path.header.frame_id = "map"

    # Contruct and publish the path message
    def publishPath(self, path):
        self.path.poses[:] = []
        for i in range(len(path)):
            p = PoseStamped()
            p.pose.position.x = (path[i][0] * self.map.info.resolution) + self.map.info.origin.position.x
            p.pose.position.y = (path[i][1] * self.map.info.resolution) + self.map.info.origin.position.y
            p.pose.position.z = 0.0
            self.path.poses.append(p)
        self.pub_path.publish(self.path)
