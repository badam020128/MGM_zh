import math
import copy
import random
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pcl2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class ZhPub2(Node):#TF Publisher node osztály
    def __init__(self):#Inicializáló függvény
        super().__init__('zh_node2')#Node név beállítása 'zh_node2'-re

        self.objects=[]

        self.objects_sub = self.create_subscription(MarkerArray, 'agent1/objects', self.objects_callback, 1)

        self.pub_path= self.create_publisher(Path, 'thirds_path', 1)

        self.timer = self.create_timer(10.0, self.timer_callback)
        self.get_logger().info('ZH Publisher 2 node has been started')


    def objects_callback(self, msg: MarkerArray):    #LaserScan üzenet fogadásakor meghívódó függvény
        self.objects=[]
        for marker in msg.markers:
            x=marker.pose.position.x
            y=marker.pose.position.y
            z=marker.pose.position.z
            self.objects.append((x, y, z))
    
    def timer_callback(self):
        if len(self.objects) < 2:
            self.get_logger().warn("Nincs elég objektum a path generálásához!  ")
            return
        
        p1, p2 = random.sample(self.objects, 2)
        
        x1, y1, z1 = p1
        x2, y2, z2 = p2

        vx = (x2 - x1) 
        vy = (y2 - y1)
        vz = (z2 - z1)

        ax=x1+(vx/3)
        ay=y1+(vy/3)
        az=z1+(vz/3)

        bx=x1+2*(vx/3)
        by=y1+2*(vy/3)
        bz=z1+2*(vz/3)

        nx=-vy
        ny=vx

        if abs(nx)<1e-9 and abs(ny)<1e-9:
            self.get_logger().warn("A kiválasztott pontok azonosnak bizonyultak!  ")
            return
        
        yaw = math.atan2(ny, nx)

        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Első harmadoló pont
        pose_a = PoseStamped()
        pose_a.header = path_msg.header
        pose_a.pose.position.x = ax
        pose_a.pose.position.y = ay
        pose_a.pose.position.z = az
        pose_a.pose.orientation.x = qx
        pose_a.pose.orientation.y = qy
        pose_a.pose.orientation.z = qz
        pose_a.pose.orientation.w = qw

        # Második harmadoló pont
        pose_b = PoseStamped()
        pose_b.header = path_msg.header
        pose_b.pose.position.x = bx
        pose_b.pose.position.y = by
        pose_b.pose.position.z = bz
        pose_b.pose.orientation.x = qx
        pose_b.pose.orientation.y = qy
        pose_b.pose.orientation.z = qz
        pose_b.pose.orientation.w = qw

        path_msg.poses.append(pose_a)
        path_msg.poses.append(pose_b)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    zh_publisher_2 = ZhPub2()
    rclpy.spin(zh_publisher_2)
    zh_publisher_2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()