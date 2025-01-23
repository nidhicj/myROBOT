import rclpy
import numpy as np
import time
from rclpy.node import Node
from std_msgs.msg import Bool
from crawler_libs.msg import Crawler
from crawler_ros2.wheels.wheel_control import WheelControl
from crawler_ros2.servos.servo_config import ServoConfig

from crawler_ros2.motion.parking import Parking
from crawler_ros2.motion.left_motion import LeftMotion
from crawler_ros2.motion.right_motion import RightMotion
from crawler_ros2.motion.straight_motion import StraightMotion
from crawler_ros2.utils.client import Client

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.voltage_sub = self.create_subscription(Crawler, 
        'crawler_velocity_pub', 
        self.callback,
        1)
        self.voltage_sub
        
        self.ready_to_drive = Bool()
        self.ready_to_drive_pub = self.create_publisher(Bool, '/listener/ready_to_drive', 1)
        
        self.client = Client()
        self.fl_servo_pos = None

        self.fl_w = WheelControl(4)
        self.fr_w = WheelControl(5)
        self.bl_w = WheelControl(6)
        self.br_w = WheelControl(7)

        self.parking = Parking()
        self.left_motion = LeftMotion()
        self.right_motion = RightMotion()
        self.straight_motion = StraightMotion()

        self.fl_servo_pos = None
        print('Controller Node')

        self.ready_to_drive_func()
    
    def ready_to_drive_func(self):
        for i in range(1,10):
            time.sleep(0.5)
            self.ready_to_drive.data = True
            self.ready_to_drive_pub.publish(self.ready_to_drive)

    def callback(self,msg):
        
        digital_voltage = msg.digital_voltage
        direction = msg.direction
        mode = msg.mode
        zero_pos = msg.zero_pos
        voltage = np.round((digital_voltage*3.8)/(65536),4)
        self.get_logger().info(f'{digital_voltage}, {voltage}, {direction}, {mode}')

        self.fl_servo_pos,self.fr_servo_pos,self.bl_servo_pos,self.br_servo_pos = self.client.call_server(1,0.0,False,mode,False)  #Just get the position
        
        # if voltage > 1.5:  # 1.4V equivalent

        if direction == 0: # moves straight
            self.straight_motion.go(digital_voltage,voltage,mode)

        if direction == 1: # moves right
            # if voltage > 1.6:
            if ((self.fl_servo_pos < 0) or (self.fr_servo_pos < 0) or (self.bl_servo_pos > 0) or (self.br_servo_pos > 0)):
                self.left_motion.turn(digital_voltage,mode, 1)
                self.get_logger().error("angle < 0 so still following left speed_factor")
            
            else:
                self.right_motion.turn(digital_voltage,mode, 1)

        elif direction == -1: # moves left 
            # if voltage > 1.6:
            if ((self.fl_servo_pos > 0) or (self.fr_servo_pos > 0) or (self.bl_servo_pos < 0) or (self.br_servo_pos < 0)):
                self.right_motion.turn(digital_voltage,mode, -1)
                self.get_logger().error("angle > 0 so still following right speed_factor")
            
            else:
                self.left_motion.turn(digital_voltage,mode, -1)
        # if voltage == 0: # parking state
        #     self.parking.stop(digital_voltage)
        #     self.get_logger().info('Parking State')
        #     pass
        
        if zero_pos == True:
            _,_,_,_ = self.client.call_server(1,0.0,True,mode,True)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
