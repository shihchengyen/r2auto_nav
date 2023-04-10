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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import math
import cmath
import time

# constants
rotatechange = 0.25
speedchange = 0.15
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.1
front_angle = 90
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'


import socket
import nmap



def yaw2angle(angle):
    #maps angles from [-pi,pi] to [-180,180]
    if angle < 0:
        return 360+(angle/math.pi*180)
    return angle/math.pi*180

def find_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('8.8.8.8', 1))  # connect() for UDP doesn't send packets
    local_ip_address = s.getsockname()[0]
    ip_search = local_ip_address.split(".")[:-1]
    ip_range = ""
    for item in ip_search:
        ip_range += item
        ip_range += "."
    ip_range += "1/24"
    s.close()
    return ip_range


def find_disp_ip(ip_range):
    nm = nmap.PortScanner()
    data = nm.scan(hosts=ip_range, arguments="-sN").get('scan')

    for item in data:
        try:
            if nm[item]['addresses']['mac'] == "80:7D:3A:FC:F0:80":
                print(item)
                return item
        except:
            pass



#ip_range = find_local_ip()
#host = find_disp_ip(ip_range) #ESP32 IP in local network
#host = find_disp_ip(ip_range)
host = '192.168.34.163'
port = 80             #ESP32 Server Port    




def angle_between(p1, p2):
    print(p1,p2)
    ydiff = p2[1]-p1[1]
    xdiff = p2[0]-p1[0]
    angle = math.degrees(math.atan2(abs(ydiff),abs(xdiff)))

    if xdiff>0:
        if ydiff>0:
            angle = 90 - angle
        else:
            angle += 90
    else:
        if ydiff>0:
            angle += 270
        else:
            angle = 270-angle

    angle = (angle - 90 + 360)%360 
    angle = 360-angle

    return angle

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





class PosNav(Node):

    def __init__(self):
        super().__init__('pos_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',1)
        # self.get_logger().info('Created publisher')

        self.cmdpub = self.create_publisher(String, 'cmdpi', 5)

        self.ultrasonic = ''

        #track coords
        self.map2base_sub = self.create_subscription(
            Pose,
            'map2base',
            self.map2base_callback,
            5)
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.og = 0
        
  
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        # create subscription to track ultrasonic
        self.ultrasub = self.create_subscription(
            String,
            'ultrasonic',
            self.ultra_callback,
            5
        )
        



        

        #checkpoints
        Table1 = [(0.79,0),(1,0),(1.21,0),(1.5,0)]
        Table2 = [(0.79,0),(1,0),(1.25,0),(1.4,-0.3),(1.4,-0.5),(1.4,-0.8),(1.4,-1)]
        Table3 = [(0.5,0),(0.5,-0.3),(0.5,-0.5),(0.5,-1)]
        Table4 = [(0.5,0),(0.5,-0.4),(0.5,-0.8),(0.5,-1.2),(0.5,-1.6)]
        Table5 = [(0.5,0),(0.5,-0.5),(0.5,-1),(0.15,-1),(0.15,-1.5),(0.15,-2),(0.15,-2.5),(0.5,-2.5),(1,-2.5),(1.5,-2.5)]
        Table6 = [(0.5,0),(1,0),(1.5,0),(1.5,-0.5),(1.5,-1),(1.5,-1.8),(2,-1.8),(2.5,-1.8),(2.5,-1.3)]
        self.Tables = [Table1,Table2,Table3,Table4,Table5,Table6]

    def forwardcal(self):
        rclpy.spin_once(self)
        temp = [self.mapbase.x,self.mapbase.y]
        self.robotforward()
        time.sleep(1)
        rclpy.spin_once(self)
        self.stopbot()
        while temp == [self.mapbase.x,self.mapbase.y]:
            rclpy.spin_once(self)
            time.sleep(0.1)
        self.og = angle_between(temp,[self.mapbase.x,self.mapbase.y])

    def ultra_callback(self, msg):
        self.ultrasonic = msg.data.strip()


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def map2base_callback(self, msg):
        #self.get_logger().info('In map2basecallback')
        
        self.mapbase = msg.position
        mapbaseorientation = msg.orientation
        self.mbroll, self.mbpitch, self.mbyaw = euler_from_quaternion(mapbaseorientation.x, mapbaseorientation.y, mapbaseorientation.z, mapbaseorientation.w)

    def dispwait(self):
        self.counter1 = 0
        rclpy.spin_once(self)
        print("msg: ",self.ultrasonic, self.ultrasonic=="can in")
        counter = 0
        while self.ultrasonic == "waiting":
            time.sleep(0.1)
            rclpy.spin_once(self)
        while(True):
            rclpy.spin_once(self)
            if self.ultrasonic == "can in":
                counter = counter + 1
                rclpy.spin_once(self)
                print(counter)
                time.sleep(1)
                rclpy.spin_once(self)
                if counter >= 5:
                    return 1
            else:
                return 0


    def ultrawait(self):
        self.counter2 = 0
        rclpy.spin_once(self)
        counter = 0
        while self.ultrasonic == "waiting":
            time.sleep(0.1)
            rclpy.spin_once(self)
        while(True):
            rclpy.spin_once(self)
            if self.ultrasonic == "can out":
                counter = counter + 1
                rclpy.spin_once(self)
                print(counter)
                time.sleep(0.5)
                if counter >= 5:
                    return 1
            else:
                return 0



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
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        curr_time = time.time()
        time_passed = 0
        if abs(rot_angle)<15:
            turnflag = 0
        else:
            turnflag = 1
        #self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        #self.get_logger().info('time_diff: %f turnflag: %f' % (time_passed, turnflag))
        while(((c_change_dir * c_dir_diff) > 0) or (time_passed < 1 and turnflag)):
            #print("mbyaw:",self.mbyaw)
            
            # allow the callback functions to run
            time_passed = time.time() - curr_time
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
        print("mapbase yaw:",self.mbyaw)
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    # function to rotate the TurtleBot
    def rotatebotslow(self, rot_angle):
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
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * (rotatechange/2)
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        curr_time = time.time()
        time_passed = 0
        if abs(rot_angle)<15:
            turnflag = 0
        else:
            turnflag = 1
        #self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        #self.get_logger().info('time_diff: %f turnflag: %f' % (time_passed, turnflag))
        while(((c_change_dir * c_dir_diff) > 0) or (time_passed < 1 and turnflag)):
            
            # allow the callback functions to run
            time_passed = time.time() - curr_time
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



    def linefollowing(self):
        rclpy.spin_once(self)
        #self.get_logger().info('In linefollowing')
        # create Twist object
        if self.ultrasonic == "left":
            self.rotatebot(2)
            self.linefollowing()
        elif self.ultrasonic == "right":
            self.rotatebot(-2)
            self.linefollowing()
        elif self.ultrasonic == "back":
            self.robotbackwards()
            time.sleep(0.5)
            self.linefollowing()
        elif self.ultrasonic == "parked":
            self.stopbot()



    def robotforward(self):
        
        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        self.publisher_.publish(twist)

    
    def robotbackwards(self):
        
        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = -speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        self.publisher_.publish(twist)


    def park(self):
        
        # start moving
        twist = Twist()
        twist.linear.x = -speedchange/5
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        self.publisher_.publish(twist)


    

    def cal(self):
        
        rclpy.spin_once(self)
        self.ogyaw = self.yaw
        rclpy.spin_once(self)
        og_coords = [self.mapbase.x,self.mapbase.y]
        rclpy.spin_once(self)
        print("going forwards")
        rclpy.spin_once(self)
        time.sleep(0.5)
        self.robotforward()
        rclpy.spin_once(self)
        time.sleep(2)
        rclpy.spin_once(self)
        self.stopbot()
        rclpy.spin_once(self)
        time.sleep(1)
        rclpy.spin_once(self)
        while og_coords == [self.mapbase.x,self.mapbase.y]:
            rclpy.spin_once(self)
        self.og = angle_between(og_coords,[self.mapbase.x,self.mapbase.y])
        print("Cal angle",self.og)
        print("mapbase yaw:",self.mbyaw)
        temp = [self.mapbase.x,self.mapbase.y]
        self.robotbackwards()
        time.sleep(2)
        rclpy.spin_once(self)
        self.stopbot()
        time.sleep(2)
        while (temp == [self.mapbase.x,self.mapbase.y]):
            rclpy.spin_once(self)
        rclpy.spin_once(self)

    

    def move_coords(self, to_x, to_y):
        mindist = 100
        rclpy.spin_once(self)
        print("og angle:",yaw2angle(self.mbyaw))
        while (abs(self.mapbase.x)>20 or abs(self.mapbase.y)>20):
            rclpy.spin_once(self)
        temp = [self.mapbase.x,self.mapbase.y]
        next = angle_between(temp,[to_x,to_y])
        print("next angle:",next)
        new_angle = (next-yaw2angle(self.mbyaw)+360)%360
        print("desired rotation: ",end="")
        print(new_angle)
        self.rotatebot(new_angle)
        stop_flag = 0
        print("end angle:", yaw2angle(self.mbyaw))
        print("found")
        self.robotforward()
        overshot = 0
        while not stop_flag:
            
            time.sleep(0.1)

            rclpy.spin_once(self)

            dist = math.sqrt((self.mapbase.x-to_x)**2 + (self.mapbase.y-to_y)**2)
            print(dist,self.mapbase.x,self.mapbase.y)
            if dist < mindist:
                mindist = dist
            if dist < stop_distance:
                stop_flag = 1
                print("yes")
            elif ((dist - mindist) > 0.02 or dist>1):
                print("Overshot")
                stop_flag = 1
                overshot = 1
        self.stopbot()
        rclpy.spin_once(self)
        next = angle_between(temp,[self.mapbase.x,self.mapbase.y])
        self.og = next
        if overshot:
            self.move_coords(to_x, to_y)
        print("done")

    def setstartyaw(self):
        rclpy.spin_once(self)
        self.startyaw = self.mbyaw

    
    def move_coords_back(self, to_x, to_y):
        mindist = 100
        rclpy.spin_once(self)
        while (abs(self.mapbase.x)>20 or abs(self.mapbase.y)>20):
            rclpy.spin_once(self)
        temp = [self.mapbase.x,self.mapbase.y]
        next = angle_between([to_x,to_y],temp)
        print("next angle:",next)
        new_angle = (next-yaw2angle(self.mbyaw)+360)%360
        print("desired rotation: ",end="")
        print(new_angle)
        self.rotatebotslow(new_angle)
        stop_flag = 0
        print("found")
        self.park()
        overshot = 0
        while not stop_flag:
            time.sleep(0.1)
            rclpy.spin_once(self)
            dist = math.sqrt((self.mapbase.x-to_x)**2 + (self.mapbase.y-to_y)**2)
            print(dist,self.mapbase.x,self.mapbase.y)
            if dist < mindist:
                mindist = dist
            if dist < 0.03:
                stop_flag = 1
                print("yes")
            elif ((dist - mindist) > 0.02 or dist>1):
                print("Overshot")
                stop_flag = 1
                overshot += 1
        self.stopbot()
        rclpy.spin_once(self)
        next = angle_between(temp,[self.mapbase.x,self.mapbase.y])
        self.og = next
        if (overshot>=3):
            print("adjusting")
            self.move_coords(temp[0], temp[1])
            self.move_coords_back(to_x, to_y)
        print("done")


    def orientate(self):
        self.rotatebot(self.ogmbyaw-yaw2angle(self.mbyaw)+360)%360
        temp = [self.mapbase.x,self.mapbase.y]
        next = angle_between(temp,[to_x,to_y])
        new_angle = (self.og-next+360)%360
        print("new: ",end="")
        print(new_angle,next,self.og,self.mapbase.x,self.mapbase.y)
        self.rotatebot(new_angle)
        rclpy.spin_once(self)
        self.og = next
        print("done")

    def pick_table(self, table):

        self.robotforward()
        time.sleep(2)
        self.stopbot()
        rclpy.spin_once(self)

        coords = self.Tables[table-1]
        next = coords[0]
        while next != coords[-1]:
            self.move_coords(next[0],next[1])
            next = coords[coords.index(next)+1]
        self.move_coords(next[0],next[1])

        #if table==6:
        #    self.find_table_6()
        print("destination reached")
        #self.robotforward()
        #time.sleep(3)
        #self.stopbot()
        
        
        while not self.ultrawait():
            time.sleep(0.1)

        self.robotbackwards()
        time.sleep(2)
        self.stopbot()

        next = coords[-2]
        while next != coords[0]:
            self.move_coords(next[0],next[1])
            next = coords[coords.index(next)-1]
        self.backup()
        



    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def find_table_6(self):
            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance using scan data
                    lri = (self.laser_range[front_angles]<float(0.40)).nonzero()
                    #removes angles where distance is more than stop_distance
                    if(len(lri[0])>0):
                        self.stopbot()
                        lr2i = np.nanargmin(self.laser_range)
                        self.rotatebot(float(lr2i))
                        lri2 = (self.laser_range[front_angles]<float(0.15).nonzero())
                        if(len(lri2[0])==0):
                            self.robotforward()
                        else:
                            self.stopbot()
                    else:
                        self.robotforward()
                rclpy.spin_once(self)

    
    def backup(self):
        rclpy.spin_once(self)
        self.move_coords(0.45,0)

        '''
        self.ogx = self.mapbase.x
        self.ogy = self.mapbase.y
        self.cal()
        print("cal done")
        while(self.mapbase.x>20 or self.mapbase.y>20):
            rclpy.spin_once(self)
            print("stuck",self.mapbase.x,self.mapbase.y)
        print("mapbase clarified")
        time.sleep(1)
        rclpy.spin_once(self)
        next = angle_between([0.2,0],[self.mapbase.x,self.mapbase.y])
        new_angle = (self.og-next+360)%360
        rclpy.spin_once(self)
        #park_angle = (self.ogyaw - self.yaw + 360)%360
        print("parking:",self.og,new_angle,self.mapbase.x,self.mapbase.y)
        self.rotatebot(new_angle)
        dist = math.sqrt((self.mapbase.x-0.2)**2 + (self.mapbase.y)**2)
        mindist = 10000
        stop_flag = 0
        overshotcount = 0
        msg = String()
        msg.data = "back"
        self.cmdpub.publish(msg)
        rclpy.spin_once(self)
        self.cal()
        next = angle_between([0.2,0],[self.mapbase.x,self.mapbase.y])
        new_angle = (self.og-next+360)%360
        rclpy.spin_once(self)
        #park_angle = (self.ogyaw - self.yaw + 360)%360
        print("parking:",self.og,new_angle,self.mapbase.x,self.mapbase.y)
        self.rotatebotslow(new_angle)
        #ratiodiff = (self.mapbase.x - self.ogx)/(self.mapbase.y - self.ogy)


        while not stop_flag and not (overshotcount>=10):
            self.park()
            print("started parking 1")
            time.sleep(0.1)
            rclpy.spin_once(self)
            dist = math.sqrt((self.mapbase.x-0.2)**2 + (self.mapbase.y)**2)
            #distmoved = math.sqrt((self.mapbase.x - self.ogx)**2 + (self.mapbase.y - self.ogy)**2)
            rclpy.spin_once(self)
            #ratiodiff = (self.mapbase.x - self.ogx)/(self.mapbase.y - self.ogy)
            #distmoved = math.sqrt((self.mapbase.x - self.ogx)**2 + (self.mapbase.y - self.ogy)**2)
            if dist < 0.03:
                stop_flag = 1
            if dist<mindist:
                mindist = dist
            elif ((dist - mindist) > 0.05):
                print("overshot parking")
                rclpy.spin_once(self)
                overshotcount += 1
            elif ((self.mapbase.x<0.2) or (abs(self.mapbase.y)>0.1)):
                overshotcount = 10
            print(dist)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        self.stopbot()
        if overshotcount >= 10:
            self.move_coords(0.3,0)
            next = angle_between([0,0],[self.mapbase.x,self.mapbase.y])
            new_angle = (self.og-next+360)%360
            rclpy.spin_once(self)
            self.rotatebotslow(new_angle)

        self.stopbot()
        rclpy.spin_once(self)
        self.ogx = self.mapbase.x
        self.ogy = self.mapbase.y
        self.cal()
        print("cal done")
        while(self.mapbase.x>20 or self.mapbase.y>20):
            rclpy.spin_once(self)
            print("stuck",self.mapbase.x,self.mapbase.y)
        print("mapbase clarified")
        time.sleep(1)
        rclpy.spin_once(self)
        next = angle_between([0,0],[self.mapbase.x,self.mapbase.y])
        new_angle = (self.og-next+360)%360
        rclpy.spin_once(self)
        #park_angle = (self.ogyaw - self.yaw + 360)%360
        print("parking:",self.og,new_angle,self.mapbase.x,self.mapbase.y)
        self.rotatebotslow(new_angle)
        dist = math.sqrt((self.mapbase.x)**2 + (self.mapbase.y)**2)
        mindist = 10000
        stop_flag = 0
        overshotcount = 0
        msg = String()
        msg.data = "back"
        self.cmdpub.publish(msg)
        rclpy.spin_once(self)

        #ratiodiff = (self.mapbase.x - self.ogx)/(self.mapbase.y - self.ogy)


        while not stop_flag and not (overshotcount>=10):
            self.park()
            print("started parking2")
            time.sleep(0.1)
            rclpy.spin_once(self)
            dist = math.sqrt((self.mapbase.x)**2 + (self.mapbase.y)**2)
            #distmoved = math.sqrt((self.mapbase.x - self.ogx)**2 + (self.mapbase.y - self.ogy)**2)
            rclpy.spin_once(self)
            #ratiodiff = (self.mapbase.x - self.ogx)/(self.mapbase.y - self.ogy)
            #distmoved = math.sqrt((self.mapbase.x - self.ogx)**2 + (self.mapbase.y - self.ogy)**2)
            if dist < 0.02:
                stop_flag = 1
            if dist<mindist:
                mindist = dist
            elif ((dist - mindist) > 0.05):
                print("overshot parking")
                rclpy.spin_once(self)
                overshotcount += 1
            elif ((self.mapbase.x<0)):
                overshotcount = 10
            print(dist)

                '''
        
        self.move_coords_back(0.3,0)
        self.move_coords_back(0,0)

        self.rotatebot((yaw2angle(self.startyaw) - yaw2angle(self.mbyaw)+360)%360)

        print("done parking")
        rclpy.spin_once(self)
        self.stopbot()
        #new_angle = (self.ogangle-self.og)
        #self.rotatebot(new_angle)
        #self.og = self.ogangle
    
    def get_init_pose(self):
        rclpy.spin_once(self)
        self.ogx = self.mapbase.x
        self.ogy = self.mapbase.y
        self.robotforward()
        time.sleep(3)
        self.stopbot()
        time.sleep(3)
        while [self.ogx,self.ogy] == [self.mapbase.x,self.mapbase.y]:
            rclpy.spin_once(self)
        self.ogangle = angle_between([self.ogx,self.ogy],[self.mapbase.x,self.mapbase.y])
        self.og = self.ogangle
        temp = [self.mapbase.x,self.mapbase.y]
        self.robotbackwards()
        time.sleep(3)
        rclpy.spin_once(self)
        self.stopbot()
        time.sleep(2)
        while (temp == [self.mapbase.x,self.mapbase.y]):
            rclpy.spin_once(self)
        rclpy.spin_once(self)


def ultrastop():
    #find angle for 
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("connected")

    #initmsg = "start".encode()
    #sock.send(initmsg)

    data = sock.recv(1).decode()

    while(data != "P"):
        data = sock.recv(1).decode().strip()        

        if data == 'P':
            sock.close()
            break
    
    

    return 1



def tableinp():
    #find angle for 
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("connected")

    #initmsg = "start".encode()
    #sock.send(initmsg)

    data = sock.recv(1).decode()

    while(not data.strip().isdigit()):
        data = sock.recv(1).decode().strip()     
        print(data)   

        if data.strip().isdigit():
            sock.close()
            break
    
    

    return int(data)


def main(args=None):
    rclpy.init(args=args)

    auto_nav = PosNav()
    
    time.sleep(2)
    auto_nav.setstartyaw()
    #auto_nav.get_init_pose()
    can_status = 0
    
    table_num = 0
    while (True):
        table_num = tableinp()
        while not can_status:
            can_status = auto_nav.dispwait()
            time.sleep(0.1)
            print("can status:",can_status)
        if can_status:
            auto_nav.pick_table(table_num)
            can_status = 0
            

    auto_nav.destroy_node()
    rclpy.shutdown()

'''
    inp = "Go"
    while inp != "Stop":
        inp = input("Go/Stop/Auto/Cal/Pick/6/backup: ")
        #if inp == "Cal":
            #auto_nav.cal()
        if inp == "Pick":
            table = input("Enter table number: ")
            auto_nav.pick_table(int(table))
        elif inp == "Go":
            inc = input("Enter coordinates like x,y: ")
            inc = inc.split(",")
            #Todo: keep track of old angle
            auto_nav.move_coords(float(inc[0]),float(inc[1]))
        elif inp == "Stop":
            auto_nav.stopbot()
        elif inp == '6':
            auto_nav.find_table_6()
        elif inp == "backup":
            auto_nav.backup()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
'''


if __name__ == '__main__':
    main()
