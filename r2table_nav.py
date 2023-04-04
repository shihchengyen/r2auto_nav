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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import paho.mqtt.client as mqtt

import numpy as np
import math
import cmath
import time
import json

# constants
ROTATECHANGE = 0.1
SPEEDCHANGE = 0.05
ANGLETHRESHOLD = 0.5
OCC_BINS = [-1, 0, 100, 101]
STOP_DISTANCE = 0.08
FRONT_ANGLE = 30
FRONT_ANGLES = range(-FRONT_ANGLE,FRONT_ANGLE+1,1)
WP_FILE = "waypoints.json"
F = open(WP_FILE, 'r+')
EXISTING_WAYPOINTS = json.load(F)
IP_ADDRESS = "172.20.10.5"


table_number = -1
next_table_num = -1
# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class TableNav(Node):

    def __init__(self):
        super().__init__('table_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
      
        # initialize variables
        
        
        self.tableNumber_subscription = self.create_subscription(
            String,
            'tableNumber',
            self.tableNumber_callback,
            10)

        self.tableNumber_subscription # prevent unused variable warning
        self.tableNumber = -1;
        self.nextTableNumber = -1;
        

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        self.limitswitch_sub = self.create_subscription(
            Bool,
            'limit_switch',
            self.switch_callback,
            qos_profile_sensor_data)
        self.switch = False
        
        # create subscription for map2base
        self.map2base_sub = self.create_subscription(
            Pose,
            'map2base',
            self.map2base_callback,
            1)
        self.map2base_sub # prevent unused variable warning
        self.mapbase = {}
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.waypointMinDistance = 10000

        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

    def switch_callback(self, msg):
        self.switch = msg.data
    
    def tableNumber_callback(self, msg):
        num = int(msg.data)
        print(msg.data)
        self.get_logger().info(msg.data)
        if (num >= 1 and num <= 6):
            if (self.tableNumber):
                self.nextTableNumber = num
            else: 
                self.tableNumber = num

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def map2base_callback(self, msg):
        # self.get_logger().info('In map2basecallback')
        
        self.mapbase = msg.position
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        print("c_change: " + str(c_change))
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        print("c_change_dir: " + str(c_change_dir))
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        print("linear.x = 0")
        # set the direction to rotate
        twist.angular.z = c_change_dir * ROTATECHANGE
        print("c_change_dir: " + str(c_change_dir))
        # start rotation
        self.publisher_.publish(twist)

        print("published twist")
        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def distance_to(self, goal):
        rclpy.spin_once(self) 
        x_diff = goal['x'] - self.mapbase.x
        y_diff = goal['y'] - self.mapbase.y
        distance = math.sqrt(x_diff ** 2 + y_diff ** 2)
        self.get_logger().info('Current distance away: %f' % distance)
        return distance
            
    def angle_to(self, goal):
        rclpy.spin_once(self)
        x_diff = goal['x'] - self.mapbase.x
        y_diff = goal['y'] - self.mapbase.y
        angle_diff = math.degrees(math.atan2(y_diff, x_diff))
        self.get_logger().info('Current angle: %f' % math.degrees(self.yaw))
        self.get_logger().info('Angle difference: %f' % angle_diff)
        return angle_diff
    
    def rotate_to_angle(self, goal):
        twist = Twist()
        curr_angle_diff = angle_to(goal)
        if (curr_angle_diff > 0):
            twist.angular.z = ROTATECHANGE
        else:
            twist.angular.z = -ROTATECHANGE
        self.publisher_.publish(twist)

        while (curr_angle_diff > ANGLE_THRESHOLD):
            rclpy.spin_once(self)
            self.get_logger().info('Current angle diff: %f' % curr_angle_diff)
            curr_angle_diff = angle_to(goal)

    def moveToTable(self, table_number):
        twist = Twist()
        table_number = str(table_number)
        for index, waypoint in enumerate(EXISTING_WAYPOINTS[table_number][1::]): 
            rclpy.spin_once(self) 
            self.get_logger().info('Current waypoint target: %d' % index)
            self.rotatebot(self.angle_to(waypoint) - math.degrees(self.yaw))
            self.waypointDistance = self.distance_to(waypoint)

            self.get_logger().info('self.waypointDistance: %f' % self.waypointDistance)
            self.get_logger().info('self.waypointMinDistance: %f' % self.waypointMinDistance)
            self.get_logger().info('self.waypointDistance > STOP: %s' % str(self.waypointDistance > STOP_DISTANCE))
            self.get_logger().info('self.waypointMinDistance > waypointDistance: %s' % str(self.waypointMinDistance > self.waypointDistance))
            while (self.waypointDistance > STOP_DISTANCE and (self.waypointMinDistance <= self.waypointDistance or self.waypointMinDistance == 10000)):
                rclpy.spin_once(self)
                twist.linear.x = SPEEDCHANGE
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                self.waypointDistance = self.distance_to(waypoint)
                self.waypointMinDistance = min(self.waypointMinDistance, self.waypointDistance)
                self.get_logger().info('self.waypointDistance > STOP: %s' % str(self.waypointDistance > STOP_DISTANCE))
                self.get_logger().info('self.waypointMinDistance: %f' % self.waypointMinDistance)
                self.get_logger().info('self.waypointMinDistance > waypointDistance: %s' % str(self.waypointMinDistance <= self.waypointDistance))
            self.waypointMinDistance = 10000
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def returnFromTable(self, table_number):
        twist = Twist()
        table_number = str(table_number)
        for index, waypoint in enumerate(EXISTING_WAYPOINTS[table_number][-2::-1]): 
            rclpy.spin_once(self) 
            self.get_logger().info('Current waypoint target: %d' % index)
            self.rotatebot(self.angle_to(waypoint) - math.degrees(self.yaw))
            self.waypointDistance = self.distance_to(waypoint)
            
            while (self.waypointDistance > STOP_DISTANCE and self.waypointMinDistance > self.waypointDistance):
                rclpy.spin_once(self)
                twist.linear.x = SPEEDCHANGE
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                self.waypointDistance = self.distance_to(waypoint)
                self.waypointMinDistance = min(self.waypointMinDistance, self.waypointDistance)
            self.waypointMinDistance = 10000
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

        
    def move(self):
        twist = Twist()
        try:
            while True:
                rclpy.spin_once(self)
                '''
                #isolated navigation testing without microswitch and mqtt
                table_number = input("Enter table number: ")
                self.moveToTable(table_number)
                self.returnFromTable(table_number)
                '''
                global table_number
                print("self.switch state: %s" % str(self.switch))
                print("table_number: %s" % str(table_number))
                if (table_number != -1 and self.switch):
                    print("Met conditions")
                    # move to table
                    print("table_number: %s" % str(table_number))
                    self.moveToTable(table_number)
                    # wait until switch detects off
                    while(self.switch):
                        print("waiting for can to be removed")
                        print("self.switch state: %s" % str(self.switch))
                        rclpy.spin_once(self)
                    # return to dispenser
                    self.returnFromTable(table_number)
                    # take next table number if available
                    if (next_table_number != -1):
                        table_number = next_table_number
                        next_table_number = -1
                    # else reset to -1
                    else:
                        table_number = -1
 
        except Exception as e:
            print(e) 
            # Ctrl-c detected
        finally:
            # stop moving
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

def on_table_num(client, userdata, msg):
    global table_number 
    global next_table_num
    if (msg.payload.decode('utf-8') != ""):
        val = int(msg.payload.decode('utf-8'))
        if(val >= 1 and val <= 6):
            if table_number != -1:
                next_table_num = val
            else:
                table_number = val
    print(table_number) 
    print(next_table_num) 

def main(args=None):
    rclpy.init(args=args)

    client = mqtt.Client("Turtlebot")
    client.message_callback_add('ESP32/tableNumber', on_table_num)
    client.connect(IP_ADDRESS, 1883)
    client.loop_start()
    client.subscribe("ESP32/tableNumber", qos=1)
    table_nav = TableNav()
    table_nav.move()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    table_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
