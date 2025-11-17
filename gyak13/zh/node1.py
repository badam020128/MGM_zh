import math
import random
import rclpy
from rclpy.node import Node #figyelni típusra
from geometry_msgs.msg import Point #figyelni típusra
from nav_msgs.msg import Odometry #figyelni típusra
from std_msgs.msg import Float64 , Int32#figyelni típusra
from visualization_msgs.msg import Marker, MarkerArray #figyelni típusra
from geometry_msgs.msg import PoseStamped #figyelni típusra
from sensor_msgs.msg import LaserScan
import tf2_ros

from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import TransformStamped


class ZhPub(Node):
    def __init__(self):
        super().__init__('zh_node')
        
        self.objects=[]
        
        for i in range(1, 1001):
            x=10 * math.cos( i ) + 0.1 * i - 10
            y=10 / i - 0.5
            z=math.sqrt(x*x+y*y)
            self.objects.append((x, y, z))
        
        self.publisher_marker = self.create_publisher(MarkerArray, 'agent1/objects', 1)

        self.publisher_siknegyed = self.create_publisher(Int32, 'siknegyed', 1)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('ZH Publisher node has been started')

        self.odom_sub = self.create_subscription(Odometry, '/agent1/odom/ground_truth', self.odom_callback, 1)

    def timer_callback(self, scan_in:LaserScan):

        for idx, (x, y, z) in enumerate(self.objects):

            self.mark_array = MarkerArray()#MarkerArray üzenet létrehozása a vizualizációhoz
            mark = Marker()#új Marker üzenet létrehozása
            mark.header.frame_id = "map"#Az üzenet frame_id-jének beállítása map-ra

            mark.id = 0#azonosító beállítása
            mark.ns = "sphere"#névtér beállítása
            mark.type = Marker.SPHERE#típus beállítása gömbre
            mark.action = Marker.ADD#hozzáadás művelet beállítása

            mark.scale.x = 0.15#x méret beállítása
            mark.scale.y = 0.15#y méret beállítása
            mark.scale.z = 0.15#z méret beállítása

            mark.color.a = random.uniform(0.2, 1.0)#átlátszóság beállítása
            mark.color.r = random.uniform(0.0, 1.0)#piros szín beállítása
            mark.color.g = random.uniform(0.0, 1.0)#zöld szín beállítása
            mark.color.b = random.uniform(0.0, 1.0)

            count_pp = 0
            for (x, y, z) in self.objects:
                if x > 0.0 and y > 0.0:
                    count_pp += 1

            siknegyed_msg = Int32()
            siknegyed_msg.data = count_pp
            self.publisher_siknegyed.publish(siknegyed_msg)

    def odom_callback(self, msg: Odometry):
        self.last_odm = msg
        rx= msg.pose.pose.position.x
        ry= msg.pose.pose.position.y
        rz= msg.pose.pose.position.z

        min_dist = float('inf')
        nearest_object = None
        for (x, y, z) in self.objects:
            dist = math.sqrt((x-rx)**2 + (y-ry)**2 + (z-rz)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_object = (x, y, z)
        if nearest_object is None:
            return

        # Készítünk egy PoseStamped-et map frame-ben a legközelebbi pontról
        nearest_pose_map = PoseStamped()
        nearest_pose_map.header.frame_id = 'map'
        nearest_pose_map.header.stamp = msg.header.stamp
        nearest_pose_map.pose.position.x = nearest_object[0]
        nearest_pose_map.pose.position.y = nearest_object[1]
        nearest_pose_map.pose.position.z = nearest_object[2]
        # orientációt hagyhatjuk alapállapotban:
        nearest_pose_map.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                'agent1/base_link',  # target frame
                'map',               # source frame
                rclpy.time.Time()    # legfrissebb elérhető transzform
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF hiba (map -> agent1/base_link): {ex}")
            return

        try:
            nearest_pose_bl = do_transform_pose(nearest_pose_map, transform)
        except Exception as ex:
            self.get_logger().warn(f"do_transform_pose hiba: {ex}")
            return

        # Közzétesszük a PoseStamped-et base_link-ben:
        self.nearest_pub.publish(nearest_pose_bl)

def main(args=None):
    rclpy.init(args=args)
    zh_pub = ZhPub()
    rclpy.spin(zh_pub)
    zh_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()