import inputs
# from crawler_ros2 import CustomMsg
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
     
class JoystickPublisher(Node):

    def __init__(self):
        super().__init__('joystick_button_status')
        self.publisher_ = self.create_publisher(String, 'joystick_status', 1)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.publish_custom_msg)
        self.i = 0
        self.pads = inputs.devices.gamepads


    def publish_custom_msg(self):
        msg = String()
        events = inputs.get_gamepad()
        
        for event in events:

            if "Sync" in event.ev_type:
                continue
            # msg.ev_type = event.ev_type
            
            msg.data = f'{event.code},{event.ev_type},{event.state}'
            # msg.state = event.state

            # self.get_logger().info(f'{event.code},{event.ev_type},{event.state}')

            self.publisher_.publish(msg)
            # print(f'{msg.data} button pressed')
        if len(self.pads) == 0:

            raise Exception("Couldn't find any Gamepads!")
    
'''
Buttons and their event.codes

X = BTN_NORTH
Y = BTN_WEST
B = BTN_EAST
A = BTN_SOUTH

RB = BTN_TR
LB = BTN_TL

right toggle button  = ABS_RX and ABS_RY
left toggle button    
Mode on:
    ABS_HAT0X and ABS_HAT0Y

Mode off:
    ABS_X and ABS_Y 

RT = ABS_RZ
LT = ABS_Z

Back  = BTN_SELECT
Start = BTN_START



'''

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = JoystickPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()