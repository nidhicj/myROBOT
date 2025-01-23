
import argparse
import rclpy
from rclpy.node import Node

from crawler_srv.srv import ServosOps
from crawler_ros2.utils.client import Client

class TestClient(Node):

    def __init__(self):
        super().__init__('test_client')

        self.client = Client()

def get_boolean_input(prompt):
    while True:
        user_input = input(prompt + " (Enter 'True' or 'False'): ").strip().lower()
        if user_input == 'true':
            return True
        elif user_input == 'false':
            return False
        else:
            print("Please enter 'True' or 'False'.")

def main(args=None):

   
    parser = argparse.ArgumentParser(description='Test Client for calling server with arguments')
    parser.add_argument('--delta_angle','-d', type=float, default=0.0, help='Value for argument 2')
    parsed_args = parser.parse_args(args)
   
    mode = 1
    print(f"You're in mode {mode}")
    rclpy.init(args=args)
    test_client = TestClient()
        
    while True:
        try:
            delta_angle = float(input("Enter the delta_angle: "))
            move = get_boolean_input("Should I move?")
            print(f'Move: {move}, delta_angle: {delta_angle}')
            if delta_angle == '':
                continue
            else:
                fl_pos, fr_pos, bl_pos, br_pos =  test_client.client.call_server(1,delta_angle,move,mode,True)
                print(fl_pos, fr_pos, bl_pos, br_pos)

        except ValueError:
            print("Sorry, I didn't understand that.")
            continue
        # else:
        #     break

    test_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    # test_client = TestClient()



