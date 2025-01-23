
import time

from adafruit_pca9685 import PCA9685

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

class WHEEL:
    def __init__(self, pca: PCA9685) -> None:
        self.__pca = pca
        self._Name = ...
        self._CHNL = ...
        self.__min_DAC = 0
        self.__max_DAC = 3800
        self.__DAC_step = 5
        self.__current_dac_val: int = 0

    def get_current_dac(self):
        return self.__current_dac_val
    
    def set_current_dac(self, dac:int):
        self.__current_dac_val = dac

    def __set_dac_value(self, value):
        self.__pca.channels[self._CHNL].duty_cycle = int(value * 0xFFFF / 4095)

    def variable_speed(self, speed_factor: float):
        if self.__min_DAC <= self.__current_dac_val <= self.__max_DAC:
            speed = int(self.__current_dac_val*speed_factor)
            self.__set_dac_value(speed)
            self.__current_dac_val = speed

    def constant_speed(self):
        #if speed is None:
        if self.__min_DAC <= self.__current_dac_val <= self.__max_DAC:
                self.__set_dac_value(self.__current_dac_val)
        # elif speed:
        #     if self.__min_DAC < speed < self.__max_DAC:
        #         self.__set_dac_value(speed)
        #         self.__current_dac_val = speed

    def set_speed(self, dac:int):
        if self.__min_DAC <= self.__current_dac_val <= self.__max_DAC:
            self.__set_dac_value(dac)

    def increase_speed(
            self, 
            speed_factor: float = 1.0,
            step_size: int = None, 
            dac: int = None
            ):
        if self.__min_DAC <= self.__current_dac_val <= self.__max_DAC:
            if step_size is None and dac is None:
                self.__current_dac_val += self.__DAC_step
            elif step_size is not None and dac is None:
                delta_dac = abs(self.__current_dac_val - step_size)
                self.__current_dac_val += delta_dac
            elif step_size is None and dac is not None:
                self.__current_dac_val = dac
            speed = int(self.__current_dac_val*speed_factor)
            self.__set_dac_value(speed)
            self.__current_dac_val = speed
            print(self.__current_dac_val)

    def decrease_speed(
            self, 
            speed_factor: float = 1.0, 
            step_size: int = None
            ):
        if self.__min_DAC <= self.__current_dac_val <= self.__max_DAC:
            if step_size is None:
                self.__current_dac_val -= self.__DAC_step
            elif step_size:
                delta_dac = abs(self.__current_dac_val - step_size)
                self.__current_dac_val -= delta_dac
            speed = int(self.__current_dac_val*speed_factor)
            self.__set_dac_value(speed)
            self.__current_dac_val = speed
        
    def stop(self):
        self.__set_dac_value(0)


class WHEEL_FL(WHEEL):
    def __init__(self, pca) -> None:
        super().__init__(pca = pca)
        self._Name = "WHEEL_FL"
        self._CHNL = 4

class WHEEL_FR(WHEEL):
    def __init__(self, pca) -> None:
        super().__init__(pca = pca)
        self._Name = "WHEEL_FR"
        self._CHNL = 5

class WHEEL_BL(WHEEL):
    def __init__(self, pca) -> None:
        super().__init__(pca = pca)
        self._Name = "WHEEL_BL"
        self._CHNL = 6

class WHEEL_BR(WHEEL):
    def __init__(self, pca) -> None:
        super().__init__(pca = pca)
        self._Name = "WHEEL_BR"
        self._CHNL = 7


if __name__ == '__main__':
    FREQUENCY = 280
    REFERENCE_CLOCK_SPEED = 25713193
    # Initialize I2C bus
    i2c = busio.I2C(SCL, SDA)

    # Creating a PCA9685 class instance.
    pca = PCA9685(
        i2c,
        address=0x40,
        reference_clock_speed=REFERENCE_CLOCK_SPEED
        )

    pca.frequency = FREQUENCY

    WHEEL_fl = WHEEL_BR(pca=pca)
    print(WHEEL_fl._CHNL)

    try:
        while True:
            for i in range(0,4000,5):
                WHEEL_fl.increase_speed()
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass 
        #WHEEL_fl.increase_speed(0)
