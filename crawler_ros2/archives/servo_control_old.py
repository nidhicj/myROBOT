import time

import binascii
from canopen import Network


network1 = Network()
network1.connect(channel='can0', bustype='socketcan')
# # # # This will attempt to read an SDO from nodes 1 - 127
network1.scanner.search()
time.sleep(0.05)


network2 = Network()
network2.connect(channel='can1', bustype='socketcan')
# # # # This will attempt to read an SDO from nodes 1 - 127
network2.scanner.search()
time.sleep(0.05)


SERVO_SPEED:int = 5556*3  # cnt/s
SERVO_ACCELERATION:int = 5556*3 # cnt/s2
SERVO_DECELERATION:int = 5556*3 # cnt/s2


class SERVO:
    def __init__(self,
                 name: str,
                 node_id: int,
                 zero_pos: int = 0,
                 actuation_range: int = 60,
                 network: Network = network1
                 )-> None:
        self.name: str = name
        self.node_id = node_id
        self.network = network
        self.zero_pos = zero_pos
        print(self.zero_pos)
        self.actuation_range = actuation_range
        self.min_limit:int = self.ang_to_cnt((self.zero_pos - (self.actuation_range/2)))
        self.max_limit:int = self.ang_to_cnt((self.zero_pos + (self.actuation_range/2)))
        self.node = self.__connect()
        status:bool = self.__initialize()
        if status:
            print(f"Servo {self.name} initialized.")

    def __initialize(self)-> bool:
        """ Initializes the servo motor to wheel zero position.
        1. Set profile mode: profile position mode (1)
        2. Set profile parameters: speed, acceleration, deceleration
        3. Enable the motor: 
            - Immediate update mode selected: Will not wait for the current
                ongoing motion command to complete if a new motion command is sent.
        4. Set the target position to zero position and wait for the motor to reach
            the zero position.

        Returns:
            _type_: bool
            _value_: True if the motor is initialized successfully, False otherwise.
        """
        self.__set_profile_mode()
        time.sleep(1)
        self.__set_profile_params()
        time.sleep(1)
        self.enable()
        # self.__pos(deg=self.zero_pos)
        
        # current_pos = round(self.get_current_pos())
        # while current_pos != self.zero_pos:
        #     time.sleep(1)
        #     current_pos = round(self.get_current_pos())
        return True

    def __connect(self)-> object:
        """ Checks if the node is present on the network and establishes 
            a connection with the node.

        Raises:
            NodeNotFound: If the specified node is not found on the network.

        Returns:
            object: Returns the node object if the node is found on the network.
        """
        if self.node_id in  self.network.scanner.nodes:
            node = self.network.add_node(self.node_id)
            print(f'Node {self.node_id} added to the network.')
        else:
            raise NodeNotFound(f"Node {self.node_id} not found on network.")
        time.sleep(0.5)
        return node
    
    def move(self, deg: int)-> None:
        """ 
        Move the servo to the specified angle. It is necessary to set profile
        params every time before sending a new motion to the servo motor.

        Args:
            deg (int): Specifies the angle in degrees to which the servo motor
                should move.
        """
        self.__set_profile_params()
        self.__pos(deg=deg)
        
    def stop_node(self)-> None:
        """ Stops the servo motor motor node.
        """
        self.node.nmt.send_command(0x2)
        print(f"Node {self.node_id} stopped !!")

    def start_node(self)-> None:
        """ Starts the servo motor node.
        """
        self.node.nmt.send_command(0x1)
        print(f"Node {self.node_id} started !!")

    def reset_communication(self)-> None:
        """ Resets the communication with the servo motor node.
        """
        self.node.nmt.send_command(0x82)
        print(f"Node {self.node_id} communication reset !!")

    def get_pos(self)-> float:
        """ Returns the current position of the servo motor in degrees.
            Servo motor position is read from the servo motor's encoder and
            converted to degrees.

        Returns:
            float: Current position of the servo motor in degrees.
        """
        
        def hexa_to_decimal(hexa: str)-> int:
            """Converts hexadecimal to decimal. Following the little endian format.

            Args:
                hexa (str): Hexadecimal string. 

            Returns:
                int: Decimal value of the hexadecimal string.
            """
            pairs = [hexa[i:i+2] for i in range(0, len(hexa), 2)]
            pairs.reverse()
            hexa_decimal = ''.join(pairs)
            decimal = int(hexa_decimal,16)
            return decimal
        
        def cnt_to_angle(cnt)-> float:
            """Converts the encoder counts to angle in degrees. 
            Here, 2^19 counts correspond to 360 degrees. 
            Each count corresponds to (360/2^19) degrees.
            This is the resolution of the servo motor.
            """
            ang = (360*cnt)/(2**19)
            return ang

        response = self.node.sdo.upload(
                index = 0x6064,
                subindex = 0x00
            )
        
        hexa = binascii.hexlify(response).decode('utf-8')
        cnt = hexa_to_decimal(hexa = hexa)
        angle = cnt_to_angle(cnt)
        return round(angle,3)


    def __set_profile_mode(self, mode:int = 1)-> None:
        """ Sets the profile mode of the servo motor. As per the datasheet,
        mode 1 corresponds to profile position mode.

        Args:
            mode (int, optional): . Defaults to 1.

        Raises:
            ProfileSetFailed: _description_
        """
        if mode == 1: # profile positon mode
            self.node.sdo.download(
                index = 0x6060,
                subindex = 0x00,
                data = int(1).to_bytes(1, 'little')
            )

            response = self.node.sdo.upload(
                index = 0x6061,
                subindex = 0x00
            )

            if int.from_bytes(response, 'little') == 1:
                print(f"Node {self.node_id} profile is set to position mode")
            else:
                raise ProfileSetFailed("Node {self.node_id} profile setting failed")


    def __set_profile_params(self)-> None:
        """ Sets the profile parameters of the servo motor. Profile parameters
        include speed, acceleration and deceleration.
        """

        # set speed/velocity
        speed_data = SERVO_SPEED.to_bytes(4, 'little')

        self.node.sdo.download(
                index = 0x6081,
                subindex = 0x00,
                data = speed_data
            )
        
        # set acceleration
        acc_data = SERVO_ACCELERATION.to_bytes(4, 'little')
        self.node.sdo.download(
                index = 0x6083,
                subindex = 0x00,
                data = acc_data
            )
        
        # set deceleration
        dec_data = SERVO_DECELERATION.to_bytes(4, 'little')
        self.node.sdo.download(
                index = 0x6084,
                subindex = 0x00,
                data = dec_data
            )
        
        # clear motor error report (no idea - following seq)
        err_data = int(128).to_bytes(2, 'little')
        self.node.sdo.download(
                index = 0x6040,
                subindex = 0x00,
                data = err_data
            )


    def enable(self)-> None:
        """ Enables the servo motor to accept motion commands (Immediate update mode).
        """

        msg = [(0x6040, 0x00,38),# set to 38
                (0x6040, 0x00,39),# set to 39
                (0x6040, 0x00,47), #set to 47
            ]
        
        for cmd in msg:
            if len(str(cmd[2]))==2:
                size = 2
            else:
                size = 4
            dat = int(cmd[2]).to_bytes(size, 'little')
            self.node.sdo.download(
                index = cmd[0],
                subindex = cmd[1],
                data = dat
            )


    def ang_to_cnt(self, ang:float)-> int:
        """ Converts the angle in degrees to encoder counts. 

        Args:
            ang (float): Angle in degrees.

        Returns:
            int: Encoder counts.
        """
        cnt = (ang/360)*(2**19)
        return int(cnt)

    def disable(self):
        self.node.sdo.download(
                index = 0x6040,
                subindex = 0x00,
                data = int(2).to_bytes(2, 'little')
            )
        response = self.node.sdo.upload(
                index = 0x6040,
                subindex = 0x00
            )
        time.sleep(1)
        print(response)
        


    def __pos(self, deg: int)-> None:
        """Moves the servo motor to the specified position. Takes angle in
        degrees as input and converts it to counts before sending the command.
        Also, checks if the angle is within the actuation range of the servo motor.

        Args:
            deg (int): _description_
        """
        cnt = self.ang_to_cnt(deg)
        # print(f'deg,cnt: ',deg,cnt)
        # print(f"self.min_limit:{self.min_limit}")
        # print(f"self.max_limit:{self.max_limit}")

        if self.min_limit <= cnt <= self.max_limit:
            data = cnt
 
        elif cnt < self.min_limit:
            data = self.min_limit

        elif cnt > self.max_limit:
            data = self.max_limit

        msg = [(0x607A, 0x00, data), # set target position
                (0x6040, 0x00, 63)
                ]
        
        for cmd in msg:
            if len(str(cmd[2]))==2:
                size = 2
            else:
                size = 4
            
            if cmd[2] > 0:
                dat = int(cmd[2]).to_bytes(size, 'little')
                self.node.sdo.download(
                    index = cmd[0],
                    subindex = cmd[1],
                    data = dat
                )


class NodeNotFound(Exception):
    pass 


class ProfileSetFailed(Exception):
    pass

if __name__ == '__main__':
    servo1 = SERVO(
        name = "Servo_FL",
        node_id = 1,
        zero_pos = 152.75,
        actuation_range = 60,
        network = network1
        )
    '''
    servo2 = SERVO(
        name = "Servo_FR",
        node_id = 2,
        zero_pos = 90,
        actuation_range = 60,
        network = network1
        )

    servo3 = SERVO(
        name = "Servo_BL",
        node_id = 3,
        zero_pos = 90,
        actuation_range = 60,
        network = network2
        )
    
    servo4 = SERVO(
        name = "Servo_BR",
        node_id = 4,
        zero_pos = 90,
        actuation_range = 60,
        network = network2
        )
    '''
    time.sleep(3)
    print(f"Servo_1 pos before 160 = {servo1.get_pos()}")
    servo1.move(160)
    
    time.sleep(5)
    print(f"Servo_1 pos after 160 = {servo1.get_pos()}")#|Servo_2 pos = {servo2.get_current_pos()}")

    # print("move done")

    # servo1.disable()
    # print("disable")

    # servo1.enable()
    # print("enable")
    
    
    # servo1.move(152.7)
    # time.sleep(2)
    # print(f"Servo_1 pos after 150 = {servo1.get_pos()}")


    # for i in range(360):
        # servo2.move(deg=i)
        # servo3.move(deg=i)
        # servo4.move(deg=i)
