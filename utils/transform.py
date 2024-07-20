#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from rclpy.qos import QoSProfile

def handle_turtle_pose(msg):
    static_transform_pub = node.create_publisher(TransformStamped, '/tf_static', qos_profile)

    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link_1"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    static_transform_pub.publish(t)
def main():
    rclpy.init()
    global node
    node = rclpy.create_node('transform')
    qos_profile = QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
    sub = node.create_subscription(Odometry,'/drone1/odom', handle_turtle_pose, qos_profile)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
