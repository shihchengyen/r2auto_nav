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
#
# adapted from https://github.com/Shashika007/teleop_twist_keyboard_ros2/blob/foxy/teleop_twist_keyboard_trio/teleop_keyboard.py

import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import nav_msgs.msg
# constants
rotatechange = 0.1
speedchange = 0.05


class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist,'cmd_vel',10)
        self.odom_subscription = self.create_subscription(nav_msgs.msg.Odometry,'odom',self.readKey_callback,10)
        print(self.odom_subscription)
        odom = nav_msgs.msg.Odometry()
        print(odom)
        self.roll=0
        self.pitch =0

    # def odom_callback(self,msg):
    #     position =  msg.pose.pose.position
    #     print(position)

    
        
        
# function to read keyboard input
    def readKey_callback(self):
        twist = geometry_msgs.msg.Twist()
        odom = nav_msgs.msg.Odometry()
        pos = odom.pose.pose.position
        orien = odom.pose.pose.orientation
        try:
            while True:
                # get keyboard input
                cmd_char = str(input("Keys w/x a/d s q: "))
        
                # check which key was entered
                if cmd_char == 's':
                    # stop moving
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif cmd_char == 'w':
                    # move forward
                    twist.linear.x += speedchange
                    twist.angular.z = 0.0
                elif cmd_char == 'x':
                    # move backward
                    twist.linear.x -= speedchange
                    twist.angular.z = 0.0
                elif cmd_char == 'a':
                    # turn counter-clockwise
                    twist.linear.x = 0.0
                    twist.angular.z += rotatechange
                elif cmd_char == 'd':
                    # turn clockwise
                    twist.linear.x = 0.0
                    twist.angular.z -= rotatechange
                elif cmd_char =='q':
                    pos_x = pos.x
                    orien_w = orien.w
                    print("pos_x",pos_x,"orien_w",orien_w)
                    print(odom.pose)



                # start the movement
                self.publisher_.publish(twist)
                
        except Exception as e:
            print(e)
            
		# Ctrl-c detected
        finally:
        	# stop moving
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    # mover.odom_callback()
    mover.readKey_callback()
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mover.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
