import rclpy 
from rclpy.node import Node 
from quycaros_pkg.srv import GetVariable1, SetVariable1, SetVariable2, GetVariable2 
from quycaros_pkg.msg import ControlMsg
import socket 
from functools import partial
from subprocess import call


class MainControl(Node): 

    def __init__(self): 

        super().__init__('main_control') 
        self.var2 = ['emotion'] #add variable names with 2 values
        self.msg = ControlMsg() #Control msg initialization
        self.msg.mode = 1; self.msg.mov_x = 0; self.msg.mov_y = 0
        self.msg.emo_x = 3; self.msg.emo_y = 4; self.msg.claw = 0

        #publisher and services initialization
        self.publisher_ = self.create_publisher(ControlMsg, '/control_msg', 10) 
        self.client_get1 = self.create_client(GetVariable1, 'get_variable1') 
        self.client_set1 = self.create_client(SetVariable1, 'set_variable1')
        self.client_get2 = self.create_client(GetVariable2, 'get_variable2') 
        self.client_set2 = self.create_client(SetVariable2, 'set_variable2') 
        self.get_logger().info("Main Control Node is running")

        #Socket initialization
        self.declare_parameter("Host", "192.168.211.211")
        self.host = self.get_parameter('Host').get_parameter_value().string_value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, 12345)) 
        self.sock.listen()
        self.conn, self.addr = self.sock.accept()  
        self.get_logger().info('Socket connection established')

        #send connection verification
        self.conn.sendall(b'ping')
        self.socket_listener()

        #keep checking for incoming socket connections
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
                self.handle_get(args[0]) 

            elif command == 'set': 
                self.handle_set(args[0], args[1]) 

            elif command == 'mov': 
                self.handle_mov(args[0]) 

    def handle_get(self, variable_name): 

        if (variable_name in self.var2):
            request = GetVariable2.Request() 
            request.variable_name = variable_name 
            future = self.client_get2.call_async(request) 
            future.add_done_callback(partial(self.get2_callback))
        else:
            request = GetVariable1.Request() 
            request.variable_name = variable_name 
            future = self.client_get1.call_async(request) 
            future.add_done_callback(partial(self.get1_callback)) 
            

    def get1_callback(self, future): 

        try: 
            response = future.result() 
            self.conn.sendall(f'{response.value}'.encode('utf-8')) 
            self.socket_listener()
        except Exception as e: 
            self.get_logger().error(f'Get1 service call failed {e}') 

    def get2_callback(self, future): 

        try: 
            response = future.result() 
            self.conn.sendall(f'({response.value_x},{response.value_y})'.encode('utf-8')) 
            self.socket_listener()
        except Exception as e: 
            self.get_logger().error(f'Get2 service call failed {e}') 

    def handle_set(self, variable_name, value): 

        if variable_name in self.var2:
            request = SetVariable2.Request()
            request.variable_name = variable_name
            request.value_x = int(value[0])
            request.value_y = int(value[2])
            future = self.client_set2.call_async(request) 
            future.add_done_callback(partial(self.set2_callback, variable_name=variable_name, value=value)) 
        else:
            request = SetVariable1.Request()
            request.variable_name = variable_name 
            request.value = int(value)
            future = self.client_set1.call_async(request) 
            future.add_done_callback(partial(self.set1_callback, variable_name=variable_name, value=value))

    def set1_callback(self, future, variable_name, value): 

        try: 
            response = future.result()
            if response.success:  
                match variable_name:
                    case 'claw_state':
                        self.msg.claw = int(value)
                    case 'mode':
                        self.msg.mode = int(value)     
                self.publisher_.publish(self.msg) 
                self.conn.sendall(f"Variable {variable_name} set to {value}".encode('utf-8')) 
                self.socket_listener() 
        except Exception as e: 
            self.get_logger().error(f'Set service call failed {e}') 

    def set2_callback(self, future, variable_name, value): 

        try: 
            response = future.result() 
            if response.success:  
                match variable_name:
                    case 'emotion':
                        self.msg.emo_x = int(value[0])
                        self.msg.emo_y = int(value[2])
                self.publisher_.publish(self.msg)
                self.conn.sendall(f"Variable {variable_name} set to {value}".encode('utf-8'))
                self.socket_listener()
        except Exception as e: 
            self.get_logger().error(f'Set service call failed {e}')

    def handle_mov(self, value): 
 
        self.msg.mov_x = int(value[0])
        self.msg.mov_y = int(value[2]) 
        self.publisher_.publish(self.msg) 
        self.conn.sendall(f"Mov set to {value}".encode('utf-8'))
        self.socket_listener()

def main(args=None): 

    rclpy.init(args=args) 
    node = MainControl() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown()
    

if __name__ == '__main__': 
    main() 