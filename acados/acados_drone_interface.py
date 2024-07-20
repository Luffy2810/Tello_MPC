import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
# from mpc_modular import MPCController
# from acados_linear import MPCController
from acados_external import MPCController
from cv_bridge import CvBridge
import math
from time import time
import numpy as np
import cv2
from inference import get_model

class DroneInterface(Node):
    def __init__(self):
        super().__init__('DroneInterface')
        self.publisher = self.create_publisher(Twist, '/drone1/cmd_vel', 25)
        self.M = np.array([[10,0,4.5],[20,0,4.5],[30,0,4.5],[40,0,4.5]])
        self.subscriber_tello_odom = self.create_subscription(
                    Odometry,
                    '/drone1/odom',
                    self.tello_odom_callback,
                    QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT))
        self.pred_x,self.pred_y = 480,360
        self.depth = 5
        self.subscriber_turtle_odom = self.create_subscription(
                    Odometry,
                    '/odom',
                    self.turtle_odom_callback,
                    QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT))
        self.current_desired_twist = Twist() 
        self.curr_tello_odom = Odometry()
        self.turtle_odom = Odometry()
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT))
        self.bridge = CvBridge()
        self.model = get_model(model_id="gazebo_mpc/1",api_key="8H624Am5CFU0bbi8aNQn")

        # optionally, change the confidence and overlap thresholds
        # values are percentages
        self.model.confidence = 50
        self.model.overlap = 25

        # x,y,?z,w = self.curr_tello_odom.pose.pose.orientation.x,self.curr_tello_odom.pose.pose.orientation.y,self.curr_tello_odom.pose.pose.orientation.z,self.curr_tello_odom.pose.pose.orientation.w
        # roll,pitch,yaw = self.euler_from_quaternion(x,y,z,w)
        self.tello_yaw = 0
        self.t1 = time()
        init_state = np.array([0,0,0,0])
        self.final_state = np.array([480 ,360,5])
        self.controller = MPCController(init_state,self.final_state)
        self.publish_rate = 100
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
        # print (1/(self.t1-time())) 
        self.t1 =time()
        twist = self.current_desired_twist
        x,y,z,w = self.curr_tello_odom.pose.pose.orientation.x,self.curr_tello_odom.pose.pose.orientation.y,self.curr_tello_odom.pose.pose.orientation.z,self.curr_tello_odom.pose.pose.orientation.w
        roll,pitch,self.tello_yaw = self.euler_from_quaternion(x,y,z,w)
        init_state = np.array([0,0,0,0])
        self.final_state= np.array([self.pred_x,self.pred_y,self.depth])
        self.controller.state_target = self.final_state
        self.controller.state_init = init_state
        tello_pos = np.array([self.turtle_odom.pose.pose.position.x,self.turtle_odom.pose.pose.position.y,self.turtle_odom.pose.pose.position.z])
        self.controller.M = self.M - tello_pos
        if np.linalg.norm(self.final_state-np.array([480,360,5]))>0.5:
            twist = self.controller.run()
            # print (init_state)
            print (twist)        
        else:
            twist = Twist() 
        self.publisher.publish(twist)
        # print (1/(time()-self.t1))
        
    def tello_odom_callback(self, msg):  
        self.curr_tello_odom = msg
    def turtle_odom_callback(self, msg):  
        self.turtle_odom = msg
        

    def image_callback(self, msg):
        # try:
            # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        t1 = time()
        results = self.model.infer(cv_image)[0]
        print ("detection time: ",1/(time()-t1))
        try:
            self.pred_x,self.pred_y = (results.predictions[0].x,results.predictions[0].y)
            self.depth = self.get_depth()
            x,y,z = self.convert_2D_3D((self.pred_x,self.pred_y),self.depth)
            self.pos = [x,y,z]
        except Exception as e:
            print ("detection error: ",e)
        # Display the image
        cv2.imshow("Drone 1 Camera", cv_image)
        cv2.waitKey(1)
        # except Exception as e:
        #     self.get_logger().error(f'Error processing image: {str(e)}')

    def get_depth(self):
        x_o,y_o,z_o = self.turtle_odom.pose.pose.position.x,self.turtle_odom.pose.pose.position.y,self.turtle_odom.pose.pose.position.z
        x_c,y_c,z_c = self.curr_tello_odom.pose.pose.position.x, self.curr_tello_odom.pose.pose.position.y ,self.curr_tello_odom.pose.pose.position.z
        depth = abs((x_o-x_c)*math.cos(self.tello_yaw)) + abs((y_o-y_c)*math.sin(self.tello_yaw))
        return depth

    def convert_2D_3D(self,point,depth,fx_d=922,fy_d=922,cx_d=480,cy_d=360):
        x_d,y_d = point 
        x_3d = (x_d - cx_d) * depth / fx_d
        y_3d = (y_d - cy_d) * depth / fy_d
        z_3d = depth
        point_3d = np.array([z_3d,-x_3d,-y_3d])
        return point_3d


    def convert_3D_2D(self,point,fx_d=922,fy_d=922,cx_d=480,cy_d=360):
        z,x,y = point 
        x=x*-1
        y=y*-1
        x_2d = x*fx_d/z + cx_d
        y_2d = y*fy_d/z + cy_d
        point_2d = np.array([x_2d,y_2d])
        return point_2d

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
