import rclpy 
from rclpy.node import Node 
from quycaros_pkg.srv import GetVariable1, SetVariable1, SetVariable2, GetVariable2

class StatesServer(Node): 

    def __init__(self): 

        super().__init__('states_server') 
        self.srv_get1 = self.create_service(GetVariable1, 'get_variable1', self.get_variable1_callback) 
        self.srv_set1 = self.create_service(SetVariable1, 'set_variable1', self.set_variable1_callback) 
        self.srv_get2 = self.create_service(GetVariable2, 'get_variable2', self.get_variable2_callback) 
        self.srv_set2 = self.create_service(SetVariable2, 'set_variable2', self.set_variable2_callback) 
        self.get_logger().info("States Server Node is running!")
        self.variables = { 
            'emotion': [3,4], 
            'claw_state': 0, 
            'mode': 1
        } 

    def get_variable1_callback(self, request, response): 

        variable = request.variable_name 

        if variable in self.variables: 
            response.value = self.variables[variable]
        else: 
            self.get_logger().error(f"Variable {variable} not found") 

        return response 
    
    def get_variable2_callback(self, request, response): 

        variable = request.variable_name 

        if variable in self.variables: 
            response.value_x = self.variables[variable][0]
            response.value_y = self.variables[variable][1]
        else: 
            self.get_logger().error(f"Variable {variable} not found") 

        return response 

    def set_variable1_callback(self, request, response): 

        variable = request.variable_name 

        if variable in self.variables: 
            self.variables[variable] = request.value 
            response.success = True 
            self.get_logger().info(f"Variable {variable} set to {request.value}") 

        else: 
            response.success = False 
            self.get_logger().error(f"Variable {variable} not found") 

        return response 
    
    def set_variable2_callback(self, request, response): 

        variable = request.variable_name 

        if variable in self.variables: 
            self.variables[variable][0] = request.value_x
            self.variables[variable][1] = request.value_y 
            response.success = True 
            self.get_logger().info(f"Variable {variable} set to ({request.value_x},{request.value_y})") 

        else: 
            response.success = False 
            self.get_logger().error(f"Variable {variable} not found") 

        return response


def main(args=None): 
    rclpy.init(args=args) 
    node = StatesServer() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 