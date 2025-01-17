#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from quycaros_pkg.msg import ControlMsg
import random



class ControlMsgPublisherNode(Node):
    def __init__(self):      
        super().__init__("control_msg_publisher")
        self.control_msg_publisher_ = self.create_publisher(ControlMsg, "control_msg", 10)
        self.timer_ = self.create_timer(30.0, self.publish_control_msg)
        self.get_logger().info("Control Msg Publisher has been started")

    def publish_control_msg(self):
          list1 = [0, 1, 2, 3, 4, 5, 6]
          msg = ControlMsg()
          msg.mode = 0
          msg.mov_x = 0
          msg.mov_y = 0
          msg.emo_x = random.choice(list1)
          msg.emo_y = random.choice(list1)
          msg.cam = False
          self.control_msg_publisher_.publish(msg)
          self.get_logger().info("message published: " + str(msg))
	

def main(args=None):    
	rclpy.init(args=args)    
	node = ControlMsgPublisherNode()
	rclpy.spin(node)    
	rclpy.shutdown()

if __name__ == "__main__":    
	main()