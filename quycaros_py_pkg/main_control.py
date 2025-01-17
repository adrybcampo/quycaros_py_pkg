import rclpy 
from rclpy.node import Node 
from quycaros_pkg.srv import GetVariable, SetVariable 
from std_msgs.msg import String 
import socket 


class MainControl(Node): 

    def __init__(self): 

        super().__init__('main_control') 
        self.publisher_ = self.create_publisher(String, '/control_msg', 10) 
        self.client_get = self.create_client(GetVariable, 'get_variable') 
        self.client_set = self.create_client(SetVariable, 'set_variable') 
        self.get_logger().info("main control node is running")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.sock.bind(('localhost', 12345)) 
        self.sock.listen(1) 
        self.conn, _ = self.sock.accept() 
        self.get_logger().info('Socket connection established') 
        self.timer = self.create_timer(0.1, self.socket_listener) 

    def socket_listener(self): 

        data = self.conn.recv(1024).decode('utf-8') 

        if data: 
            command, *args = data.split(',') 
            if command == 'get': 
                self.handle_get(args[0]) 

            elif command == 'set': 
                self.handle_set(args[0], int(args[1]), int(args[2])) 

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
            self.conn.send(f'{response.value_x},{response.value_y}'.encode('utf-8')) 

        except Exception as e: 
            self.get_logger().error(f'Get service call failed {e}') 

    def handle_set(self, variable_name, value_x, value_y): 

        request = SetVariable.Request() 
        request.variable_name = variable_name 
        request.value_x = value_x 
        request.value_y = value_y 
        future = self.client_set.call_async(request) 
        future.add_done_callback(self.set_callback) 

    def set_callback(self, future): 

        try: 
            response = future.result() 
            if response.success: 
                msg = String() 
                msg.data = 'State updated' 
                self.publisher_.publish(msg) 
        except Exception as e: 
            self.get_logger().error(f'Set service call failed {e}') 

    def handle_mov(self, value): 

        msg = String() 
        msg.data = f'mov,{value}' 
        self.publisher_.publish(msg) 

def main(args=None): 

    rclpy.init(args=args) 
    node = MainControl() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 