import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String 
import serial 

 
class EmotionControl(Node): 

    def __init__(self): 
        super().__init__('emotion_control') 
        self.subscription = self.create_subscription(String, '/control_msg', self.listener_callback, 10) 
        self.serial_conn = serial.Serial('/dev/ttyUSB0', 9600) 
        self.last_emotion = [0, 0] 

 
    def listener_callback(self, msg): 
        data = msg.data.split(',') 
        if data[0] == 'emotion': 
            new_emotion = [int(data[1]), int(data[2])] 
            self.animate_emotion(self.last_emotion, new_emotion) 
            self.last_emotion = new_emotion 

 
    def animate_emotion(self, start, end): 
        # Logic to determine the path from start to end in the 7x7 matrix 
        path = self.calculate_path(start, end) 
        for point in path: 
            self.serial_conn.write(f'{point[0]},{point[1]}'.encode('utf-8')) 

 
    def calculate_path(self, start, end): 
        # Simplified logic for determining path 
        path = [start, end]  # For illustration, direct move 
        return path 

 

def main(args=None): 
    rclpy.init(args=args) 
    node = EmotionControl() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__': 
    main() 