#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math

import math
 
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
def odom_callback(msg):
    node.get_logger().info(f"Received Odometry message: {msg}")
    orientation_q = msg.pose.pose.orientation       
    x = orientation_q.x
    y = orientation_q.y
    z = orientation_q.z
    w = orientation_q.w
    # yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    print (euler_from_quaternion(x, y, z, w))
rclpy.init(args=None)

qos_profile = QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)


node = rclpy.create_node('twist_publisher')


subscriber = node.create_subscription(
    Odometry,
    '/drone1/odom',
    odom_callback,
    qos_profile # QoS profile, similar to publisher
)
rclpy.spin(node)