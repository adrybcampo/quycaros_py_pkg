import rclpy 
from rclpy.node import Node 
from quycaros_pkg.srv import GetVariable, SetVariable 

class StatesServer(Node): 

    def __init__(self): 

        super().__init__('states_server') 
        self.declare_parameters() 
        self.srv_get = self.create_service(GetVariable, 'get_variable', self.get_variable_callback) 
        self.srv_set = self.create_service(SetVariable, 'set_variable', self.set_variable_callback) 
        self.get_logger().info("Hello Py Node!")
        self.variables = { 
            'emotion': [0, 0], 
            'camera_state': False, 
            'vel_step': 1, 
            'mode': 'LF' 
        } 

    def get_variable_callback(self, request, response): 

        variable = request.variable_name 

        if variable in self.variables: 
            response.value_x = self.variables[variable][0] 
            response.value_y = self.variables[variable][1] 

        else: 
            self.get_logger().error(f"Variable {variable} not found") 

        return response 

    def set_variable_callback(self, request, response): 

        variable = request.variable_name 

        if variable in self.variables: 
            self.variables[variable] = [request.value_x, request.value_y] 
            response.success = True 
            self.get_logger().info(f"Variable {variable} set to {request.value_x}, {request.value_y}") 

        else: 
            response.success = False 
            self.get_logger().error(f"Variable {variable} not found") 

        return response 

    def declare_parameters(self): 
        self.declare_parameter('emotion', [0, 0]) 
        self.declare_parameter('camera_state', False) 
        self.declare_parameter('vel_step', 1) 
        self.declare_parameter('mode', 'LF') 

def main(args=None): 
    rclpy.init(args=args) 
    node = StatesServer() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 