import rclpy 
from rclpy.node import Node 
from quycaros_pkg.srv import GetVariable, SetVariable 
from quycaros_pkg.msg import ControlMsg
import socket 


class MainControl(Node): 

    def __init__(self): 

        super().__init__('main_control') 
        self.publisher_ = self.create_publisher(ControlMsg, '/control_msg', 10) 
        self.client_get = self.create_client(GetVariable, 'get_variable') 
        self.client_set = self.create_client(SetVariable, 'set_variable') 
        self.get_logger().info("main control node is running")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("localhost", 12345)) 
        self.sock.listen()
        self.conn, self.addr = self.sock.accept()  
        self.get_logger().info('Socket connection established')
        self.conn.sendall(b'ping')
        self.socket_listener()
        self.timer_ = self.create_timer(10.0, self.check_socket_connection)

    def check_socket_connection(self):
        try:
            self.conn.sendall(b'ping')
        except Exception as e:
            self.conn, self.addr = self.sock.accept()  
            self.get_logger().info('Socket connection established')
            self.conn.sendall(b'ping')
            self.socket_listener()


    def socket_listener(self): 

        data = self.conn.recv(1024).decode('utf-8')

        if data: 
            command, *args = data.split(' ') 
            if command == 'get': 
                self.get_logger().info(str(args[0]))
                self.handle_get(args[0]) 

            elif command == 'set': 
                self.handle_set(args[0], (args[1])) 

            elif command == 'mov': 
                self.handle_mov(args[0]) 

    def handle_get(self, variable_name): 

        request = GetVariable.Request() 
        request.variable_name = variable_name 
        future = self.client_get.call_async(request) 
        future.add_done_callback(self.get_callback) 

    def get_callback(self, future): 

        try: 
            response = future.result() 
            self.conn.sendall(f'{response.value}'.encode('utf-8')) 
            self.socket_listener()
        except Exception as e: 
            self.get_logger().error(f'Get service call failed {e}') 

    def handle_set(self, variable_name, value): 

        request = SetVariable.Request() 
        if (variable_name == 'emotion'):
            request.emo_x = value[0]
            request.emo_y = value[2]
            future = self.client_set.call_async(request) 
            future.add_done_callback(self.set_emotion_callback, value=value) 
        else:
            request.variable_name = variable_name 
            request.value = int(value)
            future = self.client_set.call_async(request) 
            future.add_done_callback(self.set_callback, variable_name=variable_name, value=value)

    def set_callback(self, future, variable_name, value): 

        try: 
            response = future.result() 
            if response.success: 
                msg = ControlMsg() 
                match variable_name:
                    case 'cam_state':
                        msg.cam = int(value) #change later
                        return 
                    case 'claw_state':
                        msg.claw = int(value)
                        return
                    case 'mode':
                        msg.mode = int(value)
                        return
                    case default:
                        return 
                self.publisher_.publish(msg) 
        except Exception as e: 
            self.get_logger().error(f'Set service call failed {e}') 
    
    def set_emotion_callback(self, future, value): 

        try: 
            response = future.result() 
            if response.success: 
                msg = ControlMsg() 
                msg.emo_x = int(value[0])
                msg.emo_y = int(value[2])
                self.publisher_.publish(msg) 
        except Exception as e: 
            self.get_logger().error(f'Set service call failed {e}')

    def handle_mov(self, value): 

        msg = ControlMsg() 
        msg.mov_x = int(value[0])
        msg.mov_y = int(value[2]) 
        self.publisher_.publish(msg) 

def main(args=None): 

    rclpy.init(args=args) 
    node = MainControl() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown()
    

if __name__ == '__main__': 
    main() 