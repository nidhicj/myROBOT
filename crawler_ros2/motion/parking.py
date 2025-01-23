
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from crawler_ros2.wheels.wheel_control import WheelControl
# from crawler_ros2.servos.servo_control import ServoControl

class Parking(Node):

    def __init__(self):
        super().__init__('parking')

        # Include initialization of all the components -> servos and wheel here

        self.fl_w = WheelControl(4) #get feedback from the wheels ()
        self.fr_w = WheelControl(5)
        self.bl_w = WheelControl(6)
        self.br_w = WheelControl(7)
        print('Parking')

    def stop(self,voltage):
        
        voltage_factor = 1
        # self.get_logger().info(f'voltage: {voltage}')
        self.fl_w.motion(voltage,voltage_factor)
        self.fr_w.motion(voltage,voltage_factor)
        self.bl_w.motion(voltage,voltage_factor)
        self.br_w.motion(voltage,voltage_factor)

def main(args=None):
    
    rclpy.init(args=args)
    parking = Parking()
    try:
        rclpy.spin(parking)
    except KeyboardInterrupt:
        pass

    parking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()