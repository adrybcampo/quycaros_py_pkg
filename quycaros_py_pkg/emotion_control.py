import rclpy 
from rclpy.node import Node 
from quycaros_pkg.msg import ControlMsg
import serial 

 
class EmotionControl(Node): 

    def __init__(self): 
        super().__init__('emotion_control') 
        self.subscription = self.create_subscription(ControlMsg, '/control_msg', self.listener_callback, 10) 
        self.current_emotion = [3,4]
        self.new_emotion = [0,0]
        self.animation = ""
        self.get_logger().info("emotion control node is running")
        self.serial_coms = serial.Serial('/dev/ttyUSB0', 9600)

 
    def listener_callback(self, msg): 
        self.new_emotion = [msg.emo_x, msg.emo_y]
        self.calculate_animation()

    def calculate_animation(self):
        dif = [self.new_emotion[0] - self.current_emotion[0], self.new_emotion[1] - self.current_emotion[1]]
        i = self.current_emotion[0]
        j = self.current_emotion[1]
        a = 0
        b = 0
        c = abs(dif[0]) + abs(dif[1])

        while (a < c):
            b = 0
            if (dif[1] > 0):
                if (j < self.new_emotion[1]):
                    j += 1
                    face = "(" + str(i) + "," + str(j) + ")"
                    self.animation = self.animation + face
                    b += 1
                if (dif[0] > 0 and i < self.new_emotion[0]):
                    i += 1
                    face = "(" + str(i) + "," + str(j) + ")"
                    self.animation = self.animation + face
                    b += 1
                elif (dif[0] < 0 and i > self.new_emotion[0]):
                    i -= 1
                    face = "(" + str(i) + "," + str(j) + ")"
                    self.animation = self.animation + face
                    b += 1
            else:
                if (dif[0] > 0 and i < self.new_emotion[0]):
                    i += 1
                    face = "(" + str(i) + "," + str(j) + ")"
                    self.animation = self.animation + face
                    b += 1
                elif (dif[0] < 0 and i > self.new_emotion[0]):
                    i -= 1
                    face = "(" + str(i) + "," + str(j) + ")"
                    self.animation = self.animation + face
                    b += 1
                if (j > self.new_emotion[1]):
                    j -= 1
                    face = "(" + str(i) + "," + str(j) + ")"
                    self.animation = self.animation + face
                    b += 1
            a += b
        self.get_logger().info(self.animation)
        data = '<EM:' + self.animation + '>'
        data_bytes = data.encode('utf-8')
        self.serial_coms.write(data_bytes)
        self.animation = ""
        self.current_emotion = self.new_emotion
        
def main(args=None): 
    rclpy.init(args=args) 
    node = EmotionControl() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__': 
    main() 