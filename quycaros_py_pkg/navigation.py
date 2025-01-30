import rclpy 
from rclpy.node import Node 
from quycaros_pkg.msg import ControlMsg
import serial 

class Navigation(Node): 

    def __init__(self): 
        super().__init__('emotion_control') 
        self.subscription = self.create_subscription(ControlMsg, '/control_msg', self.listener_callback, 10) 
        self.current_emotion = [3,4]
        self.new_emotion = [0,0]
        self.animation = ""
        self.claw_state = 0
        self.get_logger().info("navigation node is running")
        self.serial_coms = serial.Serial('/dev/ttyUSB0', 9600)

    def listener_callback(self, msg): 

        if msg.mode == 0: 
            # Teleoperation mode 
            data = '<TO:(' + msg.mov_x + msg.mov_y + ')>'
            data_bytes = data.encode('utf-8')
            self.serial_coms.write(data_bytes)

        elif msg.mode == 1: 
            # Line follower mode 
            if msg.mov_x == 0 and msg.mov_y == 0: 
                data = '<LF:s>'
            elif msg.mov_x != 0 and msg.mov_y == 0: 
                data = '<LF:m(' + msg.mov_x + ',' + (msg.emo_x+ 1) +')>'
            elif msg.mov_x == 0 and (msg.mov_y == 1 or msg.mov_y == -1): 
                direction = 'I' if msg.mov_y == -1 else 'D'
                data = '<LF:t(' + direction + ',' + (msg.emo_x+ 1) + ')>' 
            else: 
                # Invalid condition for mov_x and mov_y 
                self.get_logger().error('Invalid mov_x and mov_y combination') 
                return 
            self.get_logger().info("Message sent: " + data)
            data_bytes = data.encode('utf-8')
            self.serial_coms.write(data_bytes)

 
def main(args=None): 
    rclpy.init(args=args) 
    node = Navigation() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__': 
    main() 