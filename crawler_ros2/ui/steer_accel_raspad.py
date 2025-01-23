import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Int64
import numpy as np
from crawler_libs.msg import Crawler

class Steer_Accel(Node):
    def __init__(self):
        super().__init__('steer_accel')
        self.btn_name = ''
        self.btn_magnitude = ''
        self.subscription = self.create_subscription(
            String,
            '/input/control_msg',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.state_code = Crawler()
        self.crawler_velocity_pub = self.create_publisher(Crawler, 'crawler_velocity_pub', 1)
        self.state_code.digital_voltage = 0

        self.declare_parameter("crawler_mode",0)

    def listener_callback(self, msg):
        
        # print('from raspad: ',msg.data)
        self.direction = msg.data.split(',')[0]
        self.wheel_vtg = msg.data.split(',')[1]
        self.mode = int(msg.data.split(',')[2])
        
        # print(self.direction, self.wheel_vtg, self.mode)

        #for mode 1
        if (self.mode == 1):
            # self.get_logger().info('Crawler in Mode 1 -> wheels at 30*')
            self.state_code.mode = 1
        
        #for mode 0
        if (self.mode == 0):
            # self.get_logger().info('Crawler in Mode 0 -> wheels at 0*')
            self.state_code.mode = 0
        
        #Wheel vtg info
        self.state_code.digital_voltage = int(self.wheel_vtg) 
        # self.get_logger().info(f"vtg_in_digital: {self.state_code.digital_voltage}")

        #Direction info
        
        if (self.direction == '1'):
            # self.get_logger().info('cralwer expected to go right')
            self.state_code.direction = 1
            # self.crawler_velocity_pub.publish(self.state_code)

        if (self.direction == '-1'):
            # self.get_logger().info('cralwer expected to go left') 
            self.state_code.direction = -1
            # self.crawler_velocity_pub.publish(self.state_code)

        if (self.direction == '0'):
            # self.get_logger().info('cralwer expected to go straight')
            self.state_code.direction = 0
            # self.crawler_velocity_pub.publish(self.state_code)
        
        self.crawler_velocity_pub.publish(self.state_code)
    # def publish_custom_data(self):

def main(args=None):
    rclpy.init(args=args)
    subscriber = Steer_Accel()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
