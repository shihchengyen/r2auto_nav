# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
import numpy as np
import time

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 1
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)



class ServoTrig(Node):

    def __init__(self):
        super().__init__('auto_servo')

        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])



    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        


    def mover(self, minimal_publisher, nowtime):
        print('flag1')
        while rclpy.ok():
            if self.laser_range.size != 0:
                # check distances in front of TurtleBot and find values less
                # than stop_distance using scan data
                lri = (float(stop_distance-0.1)<self.laser_range[front_angles]<float(stop_distance+0.1)).nonzero()
		
                #removes angles where distance is more than stop_distance
                if(len(lri[0])>0):
                    #if there are values less than stop_distance, send message
                    if time.time() - nowtime >= 9:
                        #checks if 9 seconds have passed since last message
                        minimal_publisher.send_msg()
                        nowtime = time.time()
            rclpy.spin_once(self)




class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'servo', 10) 
        #Sets up publisher on channel 'servo'
        

    def send_msg(self):
        msg = String()
        msg.data = "done"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        #Send message to channel 'servo' with data 'done' when called




def main(args=None):
    #initialise instances
    nowtime = time.time()
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    auto_servo = ServoTrig()
    auto_servo.mover(minimal_publisher, nowtime)
    #callbacks
    rclpy.spin(minimal_publisher)



if __name__ == '__main__':
    main()
