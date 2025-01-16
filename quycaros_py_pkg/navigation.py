import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String 
import serial 


class Navigation(Node): 
    def __init__(self): 
        super().__init__('navigation') 
        self.subscription = self.create_subscription(String, '/control_msg', self.listener_callback, 10) 
        self.serial_conn = serial.Serial('/dev/ttyUSB1', 9600) 


    def listener_callback(self, msg): 
        data = msg.data.split(',') 
        if len(data) < 6: 
            return 

        mode = int(data[0]) 
        mov_x = int(data[1]) 
        mov_y = int(data[2]) 
        emo_y = int(data[4]) 

        if mode == 0: 
            # Teleoperation mode 
            command = f'TO:({mov_x},{mov_y})' 
            self.serial_conn.write(command.encode('utf-8')) 
        elif mode == 1: 
            # Line follower mode 
            if mov_x == 0 and mov_y == 0: 
                command = 'LF:s(0,0)' 
            elif mov_x != 0 and mov_y == 0: 
                command = f'LF:m({mov_x},{emo_y + 1})' 
            elif mov_x == 0 and mov_y in [-1, 1]: 
                direction = 'I' if mov_y == -1 else 'D' 
                command = f'LF:t({direction},{emo_y + 1})' 
            else: 
                # Invalid condition for mov_x and mov_y 
                self.get_logger().error('Invalid mov_x and mov_y combination') 
                return 
            self.serial_conn.write(command.encode('utf-8')) 

 
def main(args=None): 
    rclpy.init(args=args) 
    node = Navigation() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__': 
    main() 