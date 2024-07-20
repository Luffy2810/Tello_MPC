import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
# from mpc_modular import MPCController
from acados_claude import MPCController
import math
from time import time
import numpy as np
class DroneInterface(Node):
    def __init__(self):
        super().__init__('DroneInterface')
        self.publisher = self.create_publisher(Twist, '/drone1/cmd_vel', 25)
        self.subscriber_odom = self.create_subscription(
                    Odometry,
                    '/drone1/odom',
                    self.odom_callback,
                    QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT))
        self.current_desired_twist = Twist() 
        self.curr_odom = Odometry()
        self.publish_rate = 25
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_twist)

    def euler_from_quaternion(self,x, y, z, w):
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

    def publish_twist(self):
        t1 =time()
        twist = self.current_desired_twist
        x,y,z,w = self.curr_odom.pose.pose.orientation.x,self.curr_odom.pose.pose.orientation.y,self.curr_odom.pose.pose.orientation.z,self.curr_odom.pose.pose.orientation.w
        roll,pitch,yaw = self.euler_from_quaternion(x,y,z,w)
        init_state = np.array([self.curr_odom.pose.pose.position.x,self.curr_odom.pose.pose.position.y,yaw])
        final_state = np.array([45,0,0])
        controller = MPCController(init_state,final_state)
        if np.linalg.norm(final_state-init_state)>2:
            twist = controller.run()
            print ([self.curr_odom.pose.pose.position.x,self.curr_odom.pose.pose.position.y,yaw])
            print (twist)        
        else:
            twist = Twist() 
        self.publisher.publish(twist)
        print (1/(time()-t1))
        
    def odom_callback(self, msg):  
        self.curr_odom = msg
        
if __name__ == '__main__':
    rclpy.init(args=None)
    drone = DroneInterface()
    
    try:
        rclpy.spin(drone)
    except KeyboardInterrupt:
        pass
    finally:
        drone.destroy_node()
        rclpy.shutdown()
