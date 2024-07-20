#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile
import os

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('drone1_image_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT))
        self.bridge = CvBridge()
        
        # Video writer setup
        self.out = None
        self.frame_width = 640  # Adjust as needed
        self.frame_height = 480  # Adjust as needed
        self.fps = 30  # Frames per second
        self.output_filename = 'drone_video.mp4'
        
        # Frame saving setup
        self.frame_count = 0
        self.save_interval = 30  # Save a frame every 30 frames
        self.output_dir = 'drone_frames'
        os.makedirs(self.output_dir, exist_ok=True)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Initialize video writer if not already done
            if self.out is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.out = cv2.VideoWriter(self.output_filename, fourcc, self.fps, 
                                           (self.frame_width, self.frame_height))
            
            # Resize image if necessary
            cv_image = cv2.resize(cv_image, (self.frame_width, self.frame_height))
            
            # Write frame to video file
            self.out.write(cv_image)
            
            # Save individual frame periodically
            if self.frame_count % self.save_interval == 0:
                frame_number = self.frame_count // self.save_interval + 1
                frame_filename = f"{self.output_dir}/frame_{frame_number}.jpg"
                cv2.imwrite(frame_filename, cv_image)
            
            self.frame_count += 1
            
            # Display the image (optional)
            cv2.imshow("Drone 1 Camera", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def __del__(self):
        # Release the video writer when the object is destroyed
        if self.out is not None:
            self.out.release()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()