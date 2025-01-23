import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Int32
import yaml
from crawler_srv.srv import ServosOps
import numpy as np
from canopen import Network
from crawler_ros2.servos.servo_config import ServoConfig
from crawler_ros2.utils.load_config import LoadConfig
from canopen import Network
import time

network0 = Network()
network0.connect(channel='can0', bustype='socketcan')
network0.scanner.search()
time.sleep(1)

network1 = Network()
network1.connect(channel='can1', bustype='socketcan')
network1.scanner.search()
time.sleep(1)

SERVO_SPEED:int = 5556*4  # cnt/s
SERVO_ACCELERATION:int = 5556*4 # cnt/s2
SERVO_DECELERATION:int = 5556*4 # cnt/s2


class ServoControl(Node):

    def __init__(self):
        super().__init__('servo_control')

        self.srv = self.create_service(ServosOps, '/listener/servo_ops', self.servo_move)
        self.servo_pos_pub = self.create_publisher(String, 'servo_pos', 1)
        self.servo_pos = String()

        self.crawler_mode_pub = self.create_publisher(Int32, 'mode_crawler', 1)
        self.mode_crawler = Int32()

        self.servo_status_pub = self.create_publisher(String, 'servo_status', 1)
        self.servo_status = String()


        self.current_mode = 1
        self.config = None
        self.config = LoadConfig()
        self.offset_fl,self.offset_fr,self.offset_bl,self.offset_br = self.config.load_config(self.current_mode)
        
        self.fl_servo = ServoConfig(network0,1)
        self.fr_servo = ServoConfig(network0,2)
        self.bl_servo = ServoConfig(network1,3)
        self.br_servo = ServoConfig(network1,4)


        self.angle_fl = 0.0 
        self.angle_fr = 0.0
        self.angle_bl = 0.0
        self.angle_br = 0.0
        self.get_logger().info('Server init')
        
        # time_period = 5.0
        # self.timer = self.create_timer(time_period, self.publish_every_5sec)

    def servo_move(self, request, response):
        
        new_mode = request.mode  # Get the requested mode
        trans = request.trans
        delta_angle = request.delta_angle 
        absolute = request.absolute
        move = request.move

        if new_mode != self.current_mode:
            print('previous Mode:',self.current_mode)
            self.offset_fl,self.offset_fr,self.offset_bl,self.offset_br = self.config.load_config(new_mode)
            self.current_mode = new_mode
            print('New Mode:',self.current_mode)
        if trans == -1:
            f_trans = -1
            b_trans = 1
        else:
            f_trans = 1
            b_trans = -1

        
        fl_servo_pos = self.fl_servo.get_pos()
        fr_servo_pos = self.fr_servo.get_pos()
        bl_servo_pos = self.bl_servo.get_pos()
        br_servo_pos = self.br_servo.get_pos()
        
        if move :
            self.get_logger().info(f'Must move with delta angle: {delta_angle}')
            if absolute == False:
                
                # self.get_logger().info(f'delta_angle: {request.delta_angle}')

                # for real operation with servo 
                fl_servo_target_pos = fl_servo_pos + f_trans * delta_angle
                fr_servo_target_pos = fr_servo_pos + f_trans * delta_angle
                bl_servo_target_pos = bl_servo_pos + b_trans * delta_angle
                br_servo_target_pos = br_servo_pos + b_trans * delta_angle

                self.fl_servo.move(fl_servo_target_pos)            
                self.fr_servo.move(fr_servo_target_pos)
                self.bl_servo.move(bl_servo_target_pos)            
                self.br_servo.move(br_servo_target_pos)            
                
                
                #for simulation
                # response.position_fl = np.round((self.angle_fl + f_trans *request.delta_angle),4)
                # response.position_fr = np.round((self.angle_fr + f_trans *request.delta_angle),4)
                # response.position_bl = np.round((self.angle_bl + b_trans *request.delta_angle),4)
                # response.position_br = np.round((self.angle_br + b_trans *request.delta_angle),4)

                
            else:
                
                self.fl_servo.move(self.offset_fl + f_trans * delta_angle)
                self.fr_servo.move(self.offset_fr + f_trans * delta_angle)
                self.bl_servo.move(self.offset_bl + b_trans * delta_angle)
                self.br_servo.move(self.offset_br + b_trans * delta_angle)
            
                # -------for simulation------------
            #     response.position_fl = np.round((f_trans *request.delta_angle),4)
            #     response.position_fr = np.round((f_trans *request.delta_angle),4)
            #     response.position_bl = np.round((b_trans *request.delta_angle),4)
            #     response.position_br = np.round((b_trans *request.delta_angle),4)


            # self.angle_fl = response.position_fl
            # self.angle_fr = response.position_fr
            # self.angle_bl = response.position_bl
            # self.angle_br = response.position_br
        #-------------------------------------
            
        position_fl = float(int(self.fl_servo.get_pos() - self.offset_fl))
        position_fr = float(int(self.fr_servo.get_pos() - self.offset_fr))
        position_bl = float(int(self.bl_servo.get_pos() - self.offset_bl))
        position_br = float(int(self.br_servo.get_pos() - self.offset_br))

        response.position_fl = position_fl        
        response.position_fr = position_fr
        response.position_bl = position_bl
        response.position_br = position_br

        self.servo_pos.data = f'{position_fl},{position_fr},{position_bl},{position_br}'
        self.servo_pos_pub.publish(self.servo_pos)
        self.get_logger().info(f"delta_angle: {request.delta_angle}, Servo pos:{np.round((position_fl),3)},\t{np.round((position_fr),3)},\t{np.round((position_bl),3)},\t{np.round((position_br),3)}")

        return response
    
    def publish_every_5sec(self):
        self.mode_crawler.data = self.current_mode
        self.crawler_mode_pub.publish(self.mode_crawler)
        
        self.servo_status.data = f'{self.fl_servo.state_node()},{self.fr_servo.state_node()},{self.bl_servo.state_node()},{self.br_servo.state_node()}'
        self.servo_status_pub.publish(self.servo_status)
        
def main(args=None):
    rclpy.init(args=args)
    servo_control = ServoControl()
    servo_control.get_logger().info("Server node ready to receive requests.")
    servo_control.get_logger().info("\tfl,\tfr,\tbl,\tbr")

    executor = rclpy.executors.SingleThreadedExecutor()
    
    executor.add_node(servo_control)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # print(f'{e} occured')
        servo_control.fl_servo.disable()
        servo_control.fr_servo.disable()
        servo_control.bl_servo.disable()
        servo_control.br_servo.disable()
        servo_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()