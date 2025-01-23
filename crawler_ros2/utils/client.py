import rclpy
from rclpy.node import Node
from crawler_srv.srv import ServosOps

class Client(Node):
    def __init__(self) -> None:
        super().__init__('client')
        self.client = self.create_client(ServosOps, '/listener/servo_ops')
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')
        self.req = ServosOps.Request()
        
    def call_server(self,trans,min_angle,move,mode,absolute):

        if not self.client.service_is_ready():
            self.get_logger().info('Service not available, waiting...')
            self.client.wait_for_service()

        self.req.trans = trans
        self.req.delta_angle = min_angle
        self.req.move = move
        self.req.mode = mode
        self.req.absolute = absolute
        
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            position_fl = response.position_fl
            position_fr = response.position_fr
            position_bl = response.position_bl
            position_br = response.position_br 
            result = [position_fl, position_fr, position_bl, position_br] # Store the result
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")
        else:
            # self.get_logger().info(f"Response: {result}")
            return result