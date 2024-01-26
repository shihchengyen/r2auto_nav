from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import rclpy
class Test_Lidar(Node):
    front = 1
    def __init__(self) -> None:
        super().__init__('test_lidar')
        self.subscription = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,
                qos_profile_sensor_data)  
        
    def scan_callback(self, msg):
        # create numpy array
        print("in scan callback")
        laser_range = np.array(msg.ranges)
        positive_range = laser_range[-30:-1]
        # print(laser_range[0])
        # taken_range = np.add(taken_range, laser_range[0:16])
        other_range = (laser_range[0:30])
        taken_range = np.append(other_range , positive_range)
        taken_range[taken_range==0] = np.nan
        # find index with minimum value
        # self.front = laser_range[0]
        # print(taken_range)
        lr2i = np.nanargmin(taken_range)
        self.front = taken_range[lr2i]
    

    def print_val(self):
        while True:
            rclpy.spin_once(self)
            if self.front == 0.0:
                print("NOTHING")
            else:
                print(self.front)
    


def main(agrs = None):
    try:
        rclpy.init()
        test = Test_Lidar()
        rclpy.spin_once(test)
        test.print_val()
    except KeyboardInterrupt:
        test.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()