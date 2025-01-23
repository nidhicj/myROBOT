import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float64
from crawler_ros2.wheels.wheel_control import WheelControl
from crawler_ros2.motion.left_motion import LeftMotion
from crawler_ros2.motion.right_motion import RightMotion
from crawler_ros2.utils.client import Client

class StraightMotion(Node):

    def __init__(self):
        super().__init__('straight_motion')

        self.client = Client()
        self.fl_servo_pos = None
        self.thresh_servo_angle_fl = 0  #0*

        self.left_motion = LeftMotion()
        self.right_motion = RightMotion()

        self.fl_w = WheelControl(4)
        self.fr_w = WheelControl(5)
        self.bl_w = WheelControl(6)
        self.br_w = WheelControl(7)

        print('Straight Motion')

    def go(self,digital_voltage,voltage,mode):
        
        
        self.fl_servo_pos,self.fr_servo_pos,self.bl_servo_pos,self.br_servo_pos = self.client.call_server(1,0.0,False,mode,False)  #Just get the position
        # self.get_logger().info(f"In Straight state, {self.fl_servo_pos},{self.fr_servo_pos},{self.bl_servo_pos},{self.br_servo_pos} ")

        # if voltage > 1.5:
            # self.get_logger().info(f'digital_voltage in stright node: {digital_voltage}')
        # if ((self.fl_servo_pos == 0) or (self.fr_servo_pos == 0) or (self.bl_servo_pos == 0) or (self.br_servo_pos == 0)) :
        voltage_factor = 1
        self.fl_w.motion(digital_voltage,voltage_factor)
        self.fr_w.motion(digital_voltage,voltage_factor)
        self.bl_w.motion(digital_voltage,voltage_factor)
        self.br_w.motion(digital_voltage,voltage_factor)

        self.get_logger().info(f"Straight State")
        
        # if ((self.fl_servo_pos < self.thresh_servo_angle_fl)  or (self.fr_servo_pos < self.thresh_servo_angle_fl) or (self.bl_servo_pos > self.thresh_servo_angle_fl) or (self.br_servo_pos > self.thresh_servo_angle_fl)) :  #state transition from left to straight
        #     self.left_motion.turn(digital_voltage,mode, +1)
        #     self.get_logger().info("Left to Straight State Transition")
            

        # if ((self.fr_servo_pos > self.thresh_servo_angle_fl) or (self.fr_servo_pos > self.thresh_servo_angle_fl) or (self.bl_servo_pos < self.thresh_servo_angle_fl)  or (self.br_servo_pos < self.thresh_servo_angle_fl)):  #state transition from right to straight
        #     self.right_motion.turn(digital_voltage,mode, -1)
        #     self.get_logger().info("Right to Straight State Transition")
            
          
        # else:
        #     self.get_logger().info('Parking State')
        

def main(args=None):
    
    rclpy.init(args=args)
    straight_motion = StraightMotion()
    try:
        rclpy.spin(straight_motion)
    except KeyboardInterrupt:
        pass

    straight_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
