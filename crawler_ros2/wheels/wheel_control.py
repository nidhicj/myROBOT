import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

class WheelControl(Node ):
# Initialize I2C bus
    def __init__(self, channel):
        super().__init__('wheel_control')
        self.i2c_bus = busio.I2C(SCL, SDA)
        # Initialize PCA9685
        self.pca = PCA9685(self.i2c_bus)
        self.pca.frequency = 50
        self.channel = channel

    def motion(self,speed,speed_factor):
        
        wheel_vtg = speed * speed_factor

        if wheel_vtg > 65535:
            wheel_vtg = 65535
        if wheel_vtg < 20000:
            wheel_vtg = 20000

        # if self.channel == 4:
        #     self.get_logger().info(f'sending int: {int(wheel_vtg)} to channel {self.channel}')
        self.pca.channels[self.channel].duty_cycle = int(wheel_vtg) # * 0x7FFF
        # time.sleep(1)


