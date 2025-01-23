import rclpy
from rclpy.node import Node
from crawler_ros2.wheels.wheel_control import WheelControl
from crawler_ros2.utils.client import Client
import time
from canopen import Network
from crawler_ros2.servos.servo_config import ServoConfig
from crawler_ros2.utils.load_config import LoadConfig



class RightMotion(Node):

    def __init__(self):
        super().__init__('right_motion')
        
        self.client_fl = Client()
        
        self.fl_servo_pos = None
        self.min_voltage = 1.9
        self.min_angle = 5.0
        self.absolute_angle = 35.0
        

        self.fl_w = WheelControl(4)
        self.fr_w = WheelControl(5)
        self.bl_w = WheelControl(6)
        self.br_w = WheelControl(7)
        print('Right Motion')

    def turn(self,digital_voltage,mode, trans):
        # msg = Float64()
        # msg.data = -self.min_angle

        if mode == 0:
            speed_factor_constant = 0.0058
        if mode == 1:
            speed_factor_constant = 0.008
        
        if trans == 1 :
            self.get_logger().info("Right State")
        else: 
            self.get_logger().info("Right to Straight State Transition")
        
        self.fl_servo_pos,self.fr_servo_pos,self.bl_servo_pos,self.br_servo_pos = self.client_fl.call_server(1, 0.0,False,mode,False)  #Just get the position
        

        if ((self.fl_servo_pos > 30.0) or (self.fr_servo_pos > 30.0) or (self.bl_servo_pos < -30.0) or (self.br_servo_pos < -30.0)):
            self.fl_servo_pos,self.fr_servo_pos,self.bl_servo_pos,self.br_servo_pos = self.client_fl.call_server(1, self.absolute_angle,True,mode,True)
            self.get_logger().warn('Reached extreme end of Right wheel')

        # if ((self.fl_servo_pos == 30.0) or (self.fr_servo_pos == 30.0) or (self.bl_servo_pos == -30.0) or (self.br_servo_pos == -30.0)):
        #     self.fl_servo_pos,self.fr_servo_pos,self.bl_servo_pos,self.br_servo_pos = self.client_fl.call_server(1, self.absolute_angle,False,mode,True)
        #     self.get_logger().warn('Staying at extreme end of Right wheel')

        else :
            self.fl_servo_pos,self.fr_servo_pos,self.bl_servo_pos,self.br_servo_pos = self.client_fl.call_server(trans,self.min_angle,True,mode,False)
            # time.sleep(0.1)
            
        right_voltage_factor = -1 * speed_factor_constant * self.fl_servo_pos + 1
        left_voltage_factor = 1
        
        # min_left_voltage = ((voltage - self.min_voltage) * left_voltage_factor) + self.min_voltage 
        
        # if digital_voltage > 33000:
        self.fl_w.motion(digital_voltage,left_voltage_factor)
        self.bl_w.motion(digital_voltage,left_voltage_factor)
        self.fr_w.motion(digital_voltage,right_voltage_factor)
        self.br_w.motion(digital_voltage,right_voltage_factor)

        # else: 
        #     self.get_logger().info('Parking State')
def main(args=None):
    
    rclpy.init(args=args)
    right_motion = RightMotion()
    try:
        rclpy.spin(right_motion)
    except KeyboardInterrupt:
        pass

    right_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()