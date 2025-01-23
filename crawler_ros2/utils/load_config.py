
import yaml

class LoadConfig():
    def __init__(self):
        pass
    def load_config(self,mode):
        with open('/home/escarda/crawler_ws/src/crawler_ros2/crawler_ros2/config/sw_profile.yml', 'r') as file:
            self.config = yaml.safe_load(file)
            offset_fl = self.config[mode]['fl_sw']['servo']
            offset_fr = self.config[mode]['fr_sw']['servo']
            offset_bl = self.config[mode]['bl_sw']['servo']
            offset_br = self.config[mode]['br_sw']['servo']


            print(f'self.crawler_mode : {offset_fl}')
            print(f'self.crawler_mode : {offset_fr}')
            print(f'self.crawler_mode : {offset_bl}')
            print(f'self.crawler_mode : {offset_br}')
            
            return offset_fl,offset_fr,offset_bl,offset_br