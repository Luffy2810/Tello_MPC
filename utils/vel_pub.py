#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
import time
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('twist_publisher')

    publisher = node.create_publisher(Twist, '/drone1/cmd_vel', 10)

    while rclpy.ok():
        try:
            # Get input from the user
            # user_input = input("Enter: ").strip()
            # if user_input.lower() == 'exit':
            #     break

            # Parse the input to get linear and angular velocities
            # linear_vel, angular_vel = map(float, user_input.split())
            linear_vel, angular_vel = 0.1, 0.1
            # Create a Twist message
            twist = Twist()
            twist.linear.x = float(linear_vel)/8
            twist.angular.x = float(0)
            twist.angular.y = float(0)
            twist.angular.z = float(angular_vel)/3.14
            publisher.publish(twist)               # while True:
            # # Publish the Twist message
            #     publisher.publish(twist)
            #     node.get_logger().info(f'Published Twist message: linear={linear_vel}, angular={angular_vel}')
            # time.sleep(100)
            # twist = Twist()
            # twist.linear.x = float(0)
            # twist.angular.z = float(0)
            
            break
        except ValueError:
            node.get_logger().warn('Invalid input. Please enter two float values separated by space (e.g., 0.5 0.2).')
        except KeyboardInterrupt:
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
