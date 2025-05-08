# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from ultralytics import YOLO  # YOLO library

# Load the YOLOv8 model
model = YOLO('yolov8m.pt')


class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('image_subscriber')
        
        # Subscribe to the drone RGB camera topic
        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/x500_depth_0/link/rgb_camera_link/sensor/rgb_camera/image',  # Correct topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
   
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        # Object Detection with YOLO
        results = model.predict(current_frame, classes=[0, 2])  # 0 = person, 2 = car
        img = results[0].plot()
        
        # Show Results
        cv2.imshow('Detected Frame', img)
        cv2.waitKey(1)
  
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = ImageSubscriber()
    
    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly (optional)
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
