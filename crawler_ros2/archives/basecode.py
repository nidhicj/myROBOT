import time

import pygame

from board import SCL, SDA
import busio

from adafruit_pca9685 import PCA9685

from wheel_control import Wheel_FL, Wheel_FR, Wheel_BL , Wheel_BR
from crawler_ros2.crawler_ros2.servos.servo_control_old import SERVO_FL, SERVO_FR, SERVO_BL, SERVO_BR

# constants
#-----------------
SERVO_SLEEP_TIME: float = 0.01

SPEED1_DAC = 1800
SPEED2_DAC = 2500
SPEED3_DAC = 3200
SPEED4_DAC = 3800

FREQUENCY = 280
REFERENCE_CLOCK_SPEED = 25713193

#--------------------------------
# Initialize I2C bus
i2c = busio.I2C(SCL, SDA)

# Creating a PCA9685 class instance.
pca = PCA9685(
    i2c,
    address=0x40,
    reference_clock_speed=REFERENCE_CLOCK_SPEED
    )

pca.frequency = FREQUENCY

# initialize servos
#--------------------------------
sfl = SERVO_FL(pca=pca)
sfl.initialize()
sfr = SERVO_FR(pca=pca)
sfr.initialize()
sbl = SERVO_BL(pca=pca)
sbl.initialize()
sbr = SERVO_BR(pca=pca)
sbr.initialize()

# initialize wheels
#--------------------------------
wfl = Wheel_FL()
wfr = Wheel_FR()
wbl = Wheel_BL()
wbr = Wheel_BR()

# initialize pygame for joystick control
#--------------------------------
pygame.init()
pygame.joystick.init()


class Button:
    """ 
    Button class for joystick button control. 
    Provides a way to check if a button is pressed or not, 
    and remembers the button's previous states.
    """
    def __init__(self, joystick, button_index):
        self.joystick = joystick
        self.button_index = button_index
        self.prev_button_state = 0

    def update(self):
        pygame.event.pump()
        button_state = self.joystick.get_button(self.button_index)
        
        if button_state != self.prev_button_state:
            #time.sleep(0.1) 
            self.prev_button_state = button_state

    def get_state(self):
        return self.prev_button_state


class Controller:
    """ 
       Class for crawler motion control. Listens to joystick inputs
       and controls the crawler motion. 
    """
    def __init__(self) -> None:

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # All buttons pygame can detect with logitech F710 controller.
        self.button_names = {
            0: "A",
            1: "B",
            2: "X",
            3: "Y",
            4: "LB",
            5: "RB",
            6: "Back",
            7: "Start"
        }

        self.axes_names = {
            0: "Left Stick X",
            1: "Left Stick Y",
            2: "Left Trigger",
            3: "Right Stick X",
            4: "Right Stick Y",
            5: "Right Trigger"
        }
        
        self.button_a = Button(self.joystick, 0)
        self.button_b = Button(self.joystick, 1)
        self.button_x = Button(self.joystick, 2)
        self.button_y = Button(self.joystick, 3)
        self.button_lb = Button(self.joystick, 4)
        self.button_rb = Button(self.joystick, 5)
        self.button_back = Button(self.joystick, 6)
        self.button_start = Button(self.joystick, 7)

        #self.reset_servo_pos()

    def update_all_buttons(self):
        self.button_a.update()
        self.button_b.update()
        self.button_x.update()
        self.button_y.update()
        self.button_lb.update()
        self.button_rb.update()
        self.button_back.update()
        self.button_start.update()

    def run(self):
        """
        Main loop for crawler motion control.
        """

        wheel_dac = 0
        servo_move_status = [True, True, True, True]
        # try:
        while True:
            pygame.event.pump()

            self.update_all_buttons()

            # turn on led when driver is active
            pca.channels[8].duty_cycle = 0xffff

            if self.button_start.get_state():
                sfl.set_la_pos()
                sfr.set_la_pos()
                sbl.set_la_pos()
                sbr.set_la_pos()
                servo_move_status = [True, True, True, True]

            if self.button_a.get_state():
                if wheel_dac < SPEED1_DAC: # increase crawler speed to speed1
                    wheel_dac += 5
                    wfl.move_forward(wheel_dac)
                    wfr.move_forward(wheel_dac)
                    wbr.move_forward(wheel_dac)
                    wbl.move_forward(wheel_dac)
                    #print(f'wheel DAC: {wheel_dac}')
                elif wheel_dac == SPEED1_DAC:
                    wfl.move_forward(SPEED1_DAC)
                    wfr.move_forward(SPEED1_DAC)
                    wbr.move_forward(SPEED1_DAC)
                    wbl.move_forward(SPEED1_DAC)
                    #print(f'reached speed1: {wheel_dac}')

            if self.button_b.get_state():
                if wheel_dac < SPEED2_DAC: # increase crawler speed to speed2
                    wheel_dac += 5
                    wfl.move_forward(wheel_dac)
                    wfr.move_forward(wheel_dac)
                    wbr.move_forward(wheel_dac)
                    wbl.move_forward(wheel_dac)
                    #print(f'wheel DAC: {wheel_dac}')
                elif wheel_dac == SPEED2_DAC:
                    wfl.move_forward(SPEED2_DAC)
                    wfr.move_forward(SPEED2_DAC)
                    wbr.move_forward(SPEED2_DAC)
                    wbl.move_forward(SPEED2_DAC)
                    #print(f'reached speed2: {wheel_dac}')
                
            if self.button_x.get_state():
                if wheel_dac < SPEED3_DAC: # increase crawler speed to speed3
                    wheel_dac += 5
                    wfl.move_forward(wheel_dac)
                    wfr.move_forward(wheel_dac)
                    wbr.move_forward(wheel_dac)
                    wbl.move_forward(wheel_dac)
                    #print(f'wheel DAC: {wheel_dac}')
                elif wheel_dac == SPEED3_DAC:
                    wfl.move_forward(SPEED3_DAC)
                    wfr.move_forward(SPEED3_DAC)
                    wbr.move_forward(SPEED3_DAC)
                    wbl.move_forward(SPEED3_DAC)
                    #print(f'reached speed3: {wheel_dac}')
            
            if self.button_y.get_state():
                if wheel_dac < SPEED4_DAC: # increase crawler speed to speed4
                    wheel_dac += 5
                    wfl.move_forward(wheel_dac)
                    wfr.move_forward(wheel_dac)
                    wbr.move_forward(wheel_dac)
                    wbl.move_forward(wheel_dac)
                    #print(f'wheel DAC: {wheel_dac}')
                elif wheel_dac == SPEED4_DAC:
                    wfl.move_forward(SPEED4_DAC)
                    wfr.move_forward(SPEED4_DAC)
                    wbr.move_forward(SPEED4_DAC)
                    wbl.move_forward(SPEED4_DAC)
                    #print(f'reached speed4: {wheel_dac}')
            
            if self.button_back.get_state(): # stop crawler (cut-off signal to wheels)
                wfl.stop()
                wfr.stop()
                wbl.stop()
                wbr.stop()
                wheel_dac = 0
            
            right_trigger = self.joystick.get_axis(5)
            if right_trigger > 0.5: # reduce crawler speed
                if wheel_dac <= SPEED4_DAC and wheel_dac >= 0:
                    wheel_dac -= 5
                    wfl.move_forward(wheel_dac)
                    wfr.move_forward(wheel_dac)
                    wbr.move_forward(wheel_dac)
                    wbl.move_forward(wheel_dac)
                if wheel_dac < 0:
                    wfl.stop()
                    wfr.stop()
                    wbl.stop()
                    wbr.stop()
                    wheel_dac = 0
            
            if self.button_start.get_state():
                wfl.stop()
                wfr.stop()
                wbl.stop()
                wbr.stop()
                wheel_dac = 0
            
            if self.button_rb.get_state(): # Turn crawler right
                if all(servo_move_status):
                    servo_move_status[0] = sfl.clockwise()
                    servo_move_status[1] = sfr.clockwise()
                    servo_move_status[2] = sbl.counter_clockwise()
                    servo_move_status[3] = sbr.counter_clockwise()
                    time.sleep(SERVO_SLEEP_TIME)
                    continue
                
            elif self.button_lb.get_state(): # Turn crawler left
                if all(servo_move_status):
                    servo_move_status[0] = sfl.counter_clockwise()
                    servo_move_status[1] = sfr.counter_clockwise()
                    servo_move_status[2] = sbl.clockwise()
                    servo_move_status[3] = sbr.clockwise()
                    time.sleep(SERVO_SLEEP_TIME)
                    continue
            else: # will bring servos back to zero position always (self centering)
                if wheel_dac > SPEED1_DAC:
                    sfl_zero_status = sfl.back_to_zero()
                    sfr_zero_status = sfr.back_to_zero()
                    sbl_zero_status = sbl.back_to_zero()
                    sbr_zero_status = sbr.back_to_zero()
                    time.sleep(SERVO_SLEEP_TIME)

                    if all([sfl_zero_status,sfl_zero_status,
                            sbl_zero_status,sbr_zero_status]):
                        servo_move_status[0] = sfl_zero_status
                        servo_move_status[1] = sfr_zero_status
                        servo_move_status[2] = sbl_zero_status
                        servo_move_status[3] = sbr_zero_status

           
if __name__ == "__main__":
    controller = Controller()
    controller.run()