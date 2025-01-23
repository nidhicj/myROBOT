import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import numpy as np
# Initialize I2C bus
i2c_bus = busio.I2C(SCL, SDA)

# Initialize PCA9685
pca = PCA9685(i2c_bus)
pca.frequency = 50  # Set PWM frequency to 50 Hz

# for i in range(10000,65000):
#     vtg = (i)*3.8 / (65535)
    
#     pca.channels[4].duty_cycle = i  # 50% duty cycle  
#     # time.sleep(1)
#     pca.channels[5].duty_cycle = i
#     # time.sleep(1)
#     pca.channels[6].duty_cycle = i
#     # time.sleep(1)
#     pca.channels[7].duty_cycle = i
#     # time.sleep(1)

#     print(i, np.round((vtg),4))

pca.channels[4].duty_cycle = 0
time.sleep(1)
pca.channels[5].duty_cycle = 0
time.sleep(1)
pca.channels[6].duty_cycle = 0
time.sleep(1)
pca.channels[7].duty_cycle = 0
# Example usage: set PWM duty cycle of channel 0 to 50%

# Turn off channel 0
