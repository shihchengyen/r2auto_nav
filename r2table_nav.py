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
DOCK_DISTANCE = 0.277
DOCK_DISTANCE_CAN = 0.17 # if can is always loaded beforehand
ROTATECHANGE = 0.4
SPEEDCHANGE = 0.175
ANGLE_THRESHOLD = 0.8
STOP_DISTANCE = 0.06
RECALIBRATE = 0.6
FRONT_ANGLE = 23
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
        self.get_logger().info('goal[x]: %f' % goal['x'])
        self.get_logger().info('mapbase.x: %f' % self.mapbase.x)
        self.get_logger().info('goal[y]: %f' % goal['y'])
        self.get_logger().info('mapbase.y: %f' % self.mapbase.y)
        self.get_logger().info('x_diff: %f' % x_diff)
        self.get_logger().info('y_diff: %f' % y_diff)
        self.get_logger().info('Current distance away: %f' % distance)
        if distance > 5:
            return self.distance_to(goal)
        return distance
            
    def angle_to(self, goal):
        rclpy.spin_once(self)
        x_diff = goal['x'] - self.mapbase.x
        y_diff = goal['y'] - self.mapbase.y
        angle_diff = math.degrees(math.atan2(y_diff, x_diff))
        self.get_logger().info('Current angle: %f' % math.degrees(self.yaw))
        self.get_logger().info('Angle difference: %f' % angle_diff)
        return angle_diff
    
    def rotate_to_goal(self, goal):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

        curr_angle_diff = self.angle_to(goal)
        diff = curr_angle_diff - math.degrees(self.yaw)
        if diff > 180:
            diff -= 360
        elif diff <= -180:
            diff += 360

        self.get_logger().info('curr diff : %f' % diff)

        if (diff > 0):
            twist.angular.z += ROTATECHANGE
        else:
            twist.angular.z -= ROTATECHANGE

        self.publisher_.publish(twist)
        min_angle = abs(math.degrees(self.yaw) - curr_angle_diff)
        break_flag = False
        while (abs(math.degrees(self.yaw) - curr_angle_diff) > ANGLE_THRESHOLD):
            self.get_logger().info('curr diff : %f' %  abs(math.degrees(self.yaw) - curr_angle_diff))
            if (abs(math.degrees(self.yaw) - curr_angle_diff) < 10 and abs(twist.angular.z) == ROTATECHANGE):
                twist.angular.z = twist.angular.z * 0.25  
            min_angle = min(abs(math.degrees(self.yaw) - curr_angle_diff), min_angle)
            if (min_angle < abs(math.degrees(self.yaw) - curr_angle_diff)):
                break_flag = True
                break
            self.publisher_.publish(twist)
            rclpy.spin_once(self)
        twist.angular.z = 0.0   
        self.publisher_.publish(twist)
        if (break_flag):
            self.rotate_to_goal(goal)

    def rotate_to(self, degree):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        degree = float(degree)
        desired_position = math.degrees(self.yaw) + degree
        if (desired_position > 180):
            desired_position -= 360
        elif (desired_position < -180):
            desired_position += 360

        # self.get_logger().info('Current angle diff: %f' % curr_angle_diff)

        if (degree > 0):
            twist.angular.z += ROTATECHANGE
        else:
            twist.angular.z -= ROTATECHANGE
        self.publisher_.publish(twist)

        self.get_logger().info('degree: %s' % str(degree))
        while (abs(desired_position - math.degrees(self.yaw)) > ANGLE_THRESHOLD):
            if(abs(desired_position - math.degrees(self.yaw)) <= 10 and twist.angular.z == ROTATECHANGE):
                twist.angular.z = 0.25 * twist.angular.z
                self.publisher_.publish(twist)
            self.get_logger().info('desired - yaw: %s' % str(abs(desired_position - math.degrees(self.yaw))))
            rclpy.spin_once(self)

    def moveToTable(self, table_number):
        twist = Twist()
        table_number = str(table_number)
        twist.linear.x = -SPEEDCHANGE
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(0.3)
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        for index, waypoint in enumerate(EXISTING_WAYPOINTS[table_number][1::]): 
            rclpy.spin_once(self) 
            self.get_logger().info('Current waypoint target: %d' % index)
            self.rotate_to_goal(waypoint)
            #self.rotatebot(self.angle_to(waypoint) - math.degrees(self.yaw))
            #self.rotate_to_angle(waypoint)
            rclpy.spin_once(self) 
            twist.linear.x = SPEEDCHANGE
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.waypointDistance = self.distance_to(waypoint)
            print('published twist')

            travelled = 0
            while (self.waypointDistance > STOP_DISTANCE) :
                if(self.waypointDistance <= 2.5 * STOP_DISTANCE and twist.linear.x == SPEEDCHANGE):
                    twist.linear.x = 0.25 * SPEEDCHANGE
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                distance = self.distance_to(waypoint)
                diff = abs(self.waypointDistance - distance)
                self.waypointDistance = distance
                travelled += diff
                self.get_logger().info('distance: %s' % str(self.waypointDistance))
                self.get_logger().info('travelled: %s' % str(travelled))
                if (travelled > RECALIBRATE): 
                    #self.rotate_to_goal(waypoint) 
                    self.rotatebot(self.angle_to(waypoint) - math.degrees(self.yaw))
                    #self.rotate_to_angle(waypoint) 
                    travelled = 0
        
        if(table_number == "6"):
            self.rotate_to(270 - math.degrees(self.yaw))
            self.get_logger().info('initialising table 6')
            min_distance= 10000;
            min_degree = 360
            for angle in FRONT_ANGLES:
                self.get_logger().info('%d, %f' % (angle, self.laser_range[angle]))
                if(self.laser_range[angle] < min_distance):
                    min_degree = angle
                    min_distance = self.laser_range[angle]
            '''
            while(not math.isnan(min_degree)):
                for angle in FRONT_ANGLES:
                    self.get_logger().info('%d, %f' % (angle, self.laser_range[angle]))
                    if(self.laser_range[angle] < min_distance):
                        min_degree = angle
            combined_ranges = np.concatenate((self.laser_range[0:23], self.laser_range[-23:]), axis=None)
            while(math.isnan(combined_ranges[0])):
                rclpy.spin_once(self)
                combined_ranges = np.concatenate((self.laser_range[0:23], self.laser_range[-23:]), axis=None)
            min_index = np.argmin(combined_ranges)
            if min_index < 23:
                degree = min_index
            else:
                degree = - (23 - min_index)
            '''
            self.rotate_to(min_degree)
            twist.linear.x = SPEEDCHANGE
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            while (self.laser_range[0] > DOCK_DISTANCE + 0.02 or math.isnan(self.laser_range[0])):
                rclpy.spin_once(self) 
                if(self.laser_range[0] <= 2.5 * DOCK_DISTANCE and twist.linear.x == SPEEDCHANGE):

                    # repeat calibration
                    '''
                    min_distance= 10000;
                    min_degree = 360
                    for angle in FRONT_ANGLES:
                        self.get_logger().info('%d, %f' % (angle, self.laser_range[angle]))
                        if(self.laser_range[angle] < min_distance):
                            min_degree = angle
                            min_distance = self.laser_range[angle]
                    self.rotate_to(min_degree)
                    '''
                    twist.linear.x = 0.25 * SPEEDCHANGE
                    self.publisher_.publish(twist)
                self.get_logger().info('laser_range[0]: %s' % str(self.laser_range[0]))
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def returnFromTable(self, table_number):
        twist = Twist()
        table_number = str(table_number)
        global STOP_DISTANCE
        for index, waypoint in enumerate(EXISTING_WAYPOINTS[table_number][-2::-1]): 

            rclpy.spin_once(self) 
            self.get_logger().info('Current waypoint target: %d' % index)
            self.rotatebot(self.angle_to(waypoint) - math.degrees(self.yaw))

            #self.rotate_to_goal(waypoint)
            rclpy.spin_once(self) 
            self.waypointDistance = self.distance_to(waypoint)
            twist.linear.x = SPEEDCHANGE
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            print('published twist')

            travelled = 0
            while (self.waypointDistance > STOP_DISTANCE) :
                if(self.waypointDistance <= 2.5 * STOP_DISTANCE and twist.linear.x == SPEEDCHANGE):
                    twist.linear.x = 0.25 * SPEEDCHANGE
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                distance = self.distance_to(waypoint)
                diff = abs(self.waypointDistance - distance)
                self.waypointDistance = distance
                travelled += diff
                self.get_logger().info('travelled: %s' % str(travelled))
                if (travelled > RECALIBRATE): 
                    
                    self.rotatebot(self.angle_to(waypoint) - math.degrees(self.yaw))
                    #self.rotate_to_goal(waypoint) 
                    travelled = 0
        # set orentation to face 0 degrees and move forward until laser_range[0] hits dockdistance
        self.rotate_to(-math.degrees(self.yaw))
        twist.linear.x = 0.022
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        dock_count = 0
        while (dock_count < 10):
            if(self.laser_range[0] >= DOCK_DISTANCE or math.isnan(self.laser_range[0])):

                self.get_logger().info('docking')
                rclpy.spin_once(self)
            else:
                self.get_logger().info('laser_range[0]: %s' % str(self.laser_range[0]))
                self.get_logger().info('dock count: %s' % str(dock_count))
                dock_count += 1
            '''
        if (self.mapbase.x <= EXISTING_WAYPOINTS["1"][0]["x"]):
            twist.linear.x = 0.025
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            while (self.laser_range[0] > DOCK_DISTANCE_CAN or math.isnan(self.laser_range[0])):
                self.get_logger().info('docking with can')
                rclpy.spin_once(self)
            '''
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
                global next_table_num

                self.get_logger().info("self.switch state: %s" % str(self.switch))
                self.get_logger().info("table_number: %s" % str(table_number))
                if (table_number != -1 and self.switch):
                    self.get_logger().info("Met conditions")
                    # move to table
                    self.get_logger().info("table_number: %s" % str(table_number))
                    self.moveToTable(table_number)
                    # wait until switch detects off
                    while(self.switch):
                        self.get_logger().info("wating for can to be removed")
                        rclpy.spin_once(self)
                    # return to dispenser
                    self.returnFromTable(table_number)
                    # take next table number if available
                    # else reset to -1
                    if (next_table_num != -1):
                        table_number = next_table_num
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
            if(table_number != -1):
                next_table_num = val
            else:
                table_number= val
    print(table_number) 
    print(next_table_num) 

def main(args=None):
    rclpy.init(args=args)

    client = mqtt.Client("Turtlebot")
    client.message_callback_add('ESP32/tableNumber', on_table_num)
    client.connect(IP_ADDRESS, 1883)
    client.loop_start()
    client.subscribe("ESP32/tableNumber", qos=2)
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
