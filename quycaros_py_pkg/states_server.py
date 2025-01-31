import rclpy 
from rclpy.node import Node 
from quycaros_pkg.srv import GetVariable, SetVariable, SetEmotion

class StatesServer(Node): 

    def __init__(self): 

        super().__init__('states_server') 
        self.srv_get = self.create_service(GetVariable, 'get_variable', self.get_variable_callback) 
        self.srv_set = self.create_service(SetVariable, 'set_variable', self.set_variable_callback) 
        self.srv_set = self.create_service(SetEmotion, 'set_emotion', self.set_emotion_callback) 
        self.get_logger().info("States Server Node is running!")
        self.variables = { 
            'emotion': [3,4], 
            'cam_state': 1,
            'claw_state': 0, 
            'mode': 1
        } 

    def get_variable_callback(self, request, response): 

        variable = request.variable_name 

        if variable in self.variables: 
            response.value = self.variables[variable]
            self.get_logger().info("response sent")
        else: 
            self.get_logger().error(f"Variable {variable} not found") 

        return response 

    def set_variable_callback(self, request, response): 

        variable = request.variable_name 

        if variable in self.variables: 
            self.variables[variable] = request.value 
            response.success = True 
            self.get_logger().info(f"Variable {variable} set to {request.value}") 

        else: 
            response.success = False 
            self.get_logger().error(f"Variable {variable} not found") 

        return response 
    
    def set_emotion_callback(self, request, response): 

        self.variables['emotion'] = [request.emo_x, request.emo_y] 
        response.success = True 
        self.get_logger().info(f"Variable emotion set to ({request.emo_x}, {request.emo_y}") 
        return response 


def main(args=None): 
    rclpy.init(args=args) 
    node = StatesServer() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 