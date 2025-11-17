import rclpy
from rclpy.node import Node
import math
import random
from nav_msgs.msg import Odometry

# TF2 importok (a repó stílusában)
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.duration import Duration 
from rclpy.time import Time 

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped, Pose # Hozzáadva: Pose
class ObjectGeneratorNode(Node):
    """
    ROS 2 node, amely 1000 objektumot generál matematikai képletek alapján, 
    és közzéteszi azokat egy MarkerArray üzenetben.
    """
    def __init__(self):
        super().__init__('object_generator_node')
        
        # Publisher létrehozása az "agent1/objects" topic-ra
        self.publisher_ = self.create_publisher(MarkerArray, 'agent1/objects', 10)
        self.sub = self.create_subscription(Odometry, "agent1/odom/ground_truth", self.callback_odom, 1)
        
        # Időzítő 1000 ms (1.0 s) periódussal
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # TF buffer és listener létrehozása
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Objektumlista (MarkerArray) generálása
        self.marker_array = self._generate_markers(1000)

    def _generate_markers(self, count):
        """Létrehoz egy MarkerArray üzenetet az 'count' számú objektummal."""
        marker_array = MarkerArray()
        
        # A kérésben szereplő ellentmondás miatt (i=[1, 100] és 1000 db objektum) 
        # a 1000 objektumot preferálva a ciklust i=1-től 1000-ig futtatjuk.
        for i in range(1, count + 1):  # i = 1, 2, ..., 1000
            
            # 1. Pozíció számítása a képletek alapján (x, y, z):
            # x = 10 * cos( i ) + 0.1 * i - 10
            # y = 10 / i - 0.5
            # z = sqrt(x*x + y*y)
            
            x = 10.0 * math.cos(float(i)) + 0.1 * float(i) - 10.0
            y = 10.0 / float(i) - 0.5
            z = math.sqrt(x**2 + y**2)
            
            marker = Marker()
            
            # 2. Üzenet beállításai
            marker.header.frame_id = "map"  # Frame beállítása
            marker.ns = "generated_objects"  # Namespace
            marker.id = i  # Egyedi azonosító
            
            # Típus: Gömb (SPHERE)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Pozíció beállítása
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0 # Nincs rotáció
            
            # 3. Méret beállítása: 15 cm átmérő
            scale = 0.15 
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            
            # 4. Szín beállítása: Véletlenszerű
            marker.color.r = random.random()
            marker.color.g = random.random()
            marker.color.b = random.random()
            marker.color.a = 1.0 # Teljes átlátszatlanság
            
            # A Marker élettartamát 0-ra állítva "örökké" megjelenik, 
            # de az időzítő folyamatosan frissíti.
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            marker_array.markers.append(marker)
            
        return marker_array
    def callback_odom(self, msg):


        if not self.marker_array_msg or not self.marker_array_msg.markers:
            return

        # 1. Jármű pozíciójának kinyerése
        vehicle_pos = msg.pose.pose.position 
        
        # 2. Legközelebbi objektum megkeresése
        closest_marker_pose = None
        min_distance_sq = float('inf') # Távolság négyzetének minimalizálása (hatékonyabb)

        for marker in self.marker_array_msg.markers:
            object_pos = marker.pose.position
            
            # Távolság négyzetének számítása 3D-ben
            dx = vehicle_pos.x - object_pos.x
            dy = vehicle_pos.y - object_pos.y
            dz = vehicle_pos.z - object_pos.z 
            distance_sq = dx**2 + dy**2 + dz**2
            
                # Keresése a legközelebbi pontnak
        for i in range(len(scan_in.ranges)):
            dist = scan_in.ranges[i] 
            if dist < closest_dist:
                closest_dist = dist
                closest_dist_angle = scan_in.angle_min + i * scan_in.angle_increment

                # Publikálása a legközelebbi pontnak
                closest_point = PoseStamped()
                closest_point.header = scan_in.header
                closest_point.pose.position.x = closest_dist * math.cos(closest_dist_angle)
                closest_point.pose.position.y = closest_dist * math.sin(closest_dist_angle)
                closest_point.pose.position.z = 0.0
                closest_point.pose.orientation.w = 1.0
                self.closest_point_pub.publish(closest_point)
            return
        # --- 3. Transzformáció a cél frame-be ("agent1/base_link") ---
        target_frame = "map"
        source_frame = msg.header.frame_id
        
        # Ellenőrizzük, hogy elérhető-e a transzformáció (a repó stílusa szerint, try/except nélkül)
        if self.tf_buffer.can_transform(target_frame, source_frame, msg.header.stamp, Duration(seconds=0.1)):

            # transzformáció lekérése
            trans_scan2map = self.tf_buffer.lookup_transform(
                target_frame, 
                source_frame, 
                msg.header.stamp
            )
            
            # Pose transzformálása (a save_path.py-ban látott módon)
            pose_map = do_transform_pose(pose_local, trans_scan2map)
            # Publikálás (PoseStamped üzenetként, a frame információjával)
            pose_stamped_map = PoseStamped()
            pose_stamped_map.header.frame_id = target_frame
            pose_stamped_map.header.stamp = msg.header.stamp
            pose_stamped_map.pose = pose_map
            
            self.pub_pose.publish(pose_stamped_map)
            
        # Ha a transzformáció nem érhető el, a kód itt visszatér.

    def timer_callback(self):
        """Az időzítő visszahívása, amely közzéteszi a MarkerArray üzenetet."""
        
        # A MarkerArray fejlécének frissítése az aktuális időre
        self.marker_array.header.stamp = self.get_clock().now().to_msg()
        
        # Üzenet közzététele
        self.publisher_.publish(self.marker_array)
        # self.get_logger().info(f'Published MarkerArray with {len(self.marker_array.markers)} markers.')


def main(args=None):
    rclpy.init(args=args)
    
    object_generator_node = ObjectGeneratorNode()
    
    try:
        rclpy.spin(object_generator_node)
    except KeyboardInterrupt:
        pass
        
    # Tisztítás
    object_generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()