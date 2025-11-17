#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

# Node létrehozása
class PathCreator(Node):
    def __init__(self):
        super().__init__('odom_path')

        # kívülről átadott paraméterek
        self.declare_parameter('path_topic_name', '/path')
        self.declare_parameter('odom_topic_name', '/odom')
        self.declare_parameter('max_size', 1500)

        # paraméterek értékeinek lekérése
        path_topic_name = self.get_parameter('path_topic_name').value
        odom_topic_name = self.get_parameter('odom_topic_name').value
        self.max_size = self.get_parameter('max_size').value

        # robot utvonala
        self.path = Path()

        # publisher és subscriber létrehozása
        self.path_pub = self.create_publisher(Path, path_topic_name, 1)
        self.odom_sub = self.create_subscription(Odometry, odom_topic_name, self.odom_cb, 1)

        self.get_logger().info(
            f"Publishing path on [{path_topic_name}] from odometry [{odom_topic_name}]"
        )

    def odom_cb(self, msg: Odometry):
        # ha elérte a maximális méretet, akkor a régi pozíciók törlése
        if (len(self.path.poses) >= self.max_size) and (self.max_size > 0):
            del self.path.poses[0:int(self.max_size * 0.2)]

        # üres posestamped létrehozása
        pose = PoseStamped()
        # feltöltése az odometry adataival
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # path feltöltése headerrel
        self.path.header = msg.header
        # path feltöltése a pozícióval
        self.path.poses.append(pose)

        # Publish path
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()