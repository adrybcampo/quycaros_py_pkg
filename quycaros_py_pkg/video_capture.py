import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
import cv2 
from cv_bridge import CvBridge 

 
class VideoCapture(Node): 
    def __init__(self): 
        super().__init__('video_capture') 
        self.publisher_ = self.create_publisher(Image, '/camera', 10) 
        self.timer = self.create_timer(0.1, self.timer_callback) 
        self.cap = cv2.VideoCapture(0) 
        self.bridge = CvBridge() 


    def timer_callback(self): 
        ret, frame = self.cap.read() 
        if ret: 
            image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8") 
            self.publisher_.publish(image_message) 

 
def main(args=None): 
    rclpy.init(args=args) 
    node = VideoCapture() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 

 
if __name__ == '__main__': 
    main() 