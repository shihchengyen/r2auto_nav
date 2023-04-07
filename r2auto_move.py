import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
from math import atan2
import pickle
from math import pi
import cmath
# import RPI.GPIO as GPIO
import paho.mqtt.client as mqtt
with open("waypoints.pickle","rb") as handle:
    waypoints = pickle.load(handle)
re = 0
print(waypoints)
mapfile = 'map.txt'
speedchange = 0.05
angle_error = 2
paths = {1:[0],2:[0,2],3:[0,2],4:[0,3],5:[0,4]}
# print("in in in ")
count = 0

# quad_1 = range(0, 0.5 * pi)
# quad_2 = range (0.5 * pi, pi)
# quad_3 = range(pi, -0.5 * pi)
# quad_4 = range(-0.5 * pi, -pi)

# waypoints = {1:[-2.5769601779175844, -1.23103603909631],2:[-2.1849758576303553, -0.5775387034088549]}

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return  yaw_z # in radians

class Auto_Mover(Node):
    table = 0
    rot_q = 0.0
    orien = 0.0
    count = 0
    front = 0.0
    yaw = 0.0
    def __init__(self) -> None:
        self.x = -1
        self.y = -1
        super().__init__('auto_mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel',10)
        self.user_subscription = self.create_subscription(String,
                                                          'user',self.user_sub,10)
        self.odom_subsription = self.create_subscription(Odometry,
            'odom',
            self.odom_callback,
            10)
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)  

    def occ_callback(self,msg):
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
        print(msg)
        np.savetxt(mapfile, self.occdata)


    def odom_callback(self, msg):
        # print("callback")
        self.rot_q = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.orien = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
        
        # print(self.rot_q)
        
        # points_char = int(input("enter waypoint to travel: "))
    
        # # print("qewagdsfnc")
    
        
    def user_sub(self, msg):
        self.table = int(msg.data)
        # print("in subcriber user")    

    def scan_callback(self, msg):
        # create numpy array
        print("in scan callback")
        laser_range = np.array(msg.ranges)
        # print("break here:1")
        positive_range = laser_range[-30:-1]
        # print(laser_range[0])
        # taken_range = np.add(taken_range, laser_range[0:16])
        other_range = (laser_range[0:30])
        taken_range = np.append(other_range , positive_range)
        taken_range[taken_range==0] = np.nan
        # taken_range = list(filter(lambda x: x == 0.0,taken_range))
        # find index with minimum value
        # self.front = laser_range[0]
        # print("break here:2")
        
        if np.isnan(taken_range).all() == True:
            self.front = 3.0
            print("all NAN")
        else:
            lr2i = np.nanargmin(taken_range)
            self.front = taken_range[lr2i]
        # self.angle_go = math.radians(lr2i)
        # log the info
        # self.get_logger().info('Shortest distance at %i degrees' % lr2i)    
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = geometry_msgs.msg.Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        # self.get_logger().info('Current: %f' % math.degrees(current_yaw))
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
        twist.angular.z = c_change_dir * speedchange
        # start rotation
        print("publishing twist")
        self.publisher_.publish(twist)

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
            print(current_yaw)
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            print(c_change)
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def travelling_point(self, point):
        print("tavelling to waypoint")
        # points_char = int(input("enter waypoint to travel: "))
        twist = geometry_msgs.msg.Twist()
        # print("qewagdsfnc")
        
        # rclpy.init_node("speed_controller")
        # r = rclpy.Rate(4)
        if point == 7:
            self.goal_x = waypoints[0][0][0]
            self.goal_y = waypoints[0][0][1]
        else:
            self.goal_x = waypoints[point][0][0]
            self.goal_y = waypoints[point][0][1]
       
        theta = (atan2(self.goal_y-self.y,self.goal_x-self.x))
        inc_x = 10000000 
        degree_to_turn = math.degrees(theta - self.orien )
        sign = np.sign(degree_to_turn)
        degree_to_turn = math.degrees(theta - self.orien )
        try:

                while inc_x != 0:
                    # print("while in loop")
                    rclpy.spin_once(self)
                    # print(self.orien)
                    print(point)
                   
                    inc_x = self.goal_x - self.x
                    current_angle =  math.degrees(self.orien)
                    # print("x",self.x, "inc",inc_x)
                    inc_y = self.goal_y - self.y
                    if  abs(degree_to_turn) > angle_error:
                        degree_to_turn = math.degrees(theta - self.orien )
                        print("angle finding")
                        # self.rotatebot(degree_to_turn)
                        twist.linear.x = 0.0
                        twist.angular.z = 0.5 * sign
                        print("degree to turn", degree_to_turn)
                        print("current angle", math.degrees(self.orien))
                        print("theta", math.degrees(theta))
                        print(sign)
                        print(point)
                        print("current x", self.x)
                        print("goal", self.goal_x)
                        print("current y", self.y)
                        print("goal", self.goal_y)
                    elif point == (3 or 7) or (point ==  4 and re == 0):
                        print("for point 3 and 4, but point is:",point)
                        if  (abs(int(abs(self.goal_y)*100-2)!=int(abs(self.y)*100 )))  :
                            twist.linear.x = 0.1
                            twist.angular.z = 0.0  
                            print("current y", self.y)
                            print("goal", self.goal_y)   
                        else:
                            twist.linear.x = 0.00
                            twist.angular.z = 0.0 
                            self.publisher_.publish(twist)
                            break               
                    elif (abs(int(abs(self.goal_x)*100 -2)!=int(abs(self.x)*100 ))):
                        # and abs((int(abs(self.goal_y)*100-2)-int(abs(self.y)*100))<= 3)
                        print("moving")
                        print("current x", self.x)
                        print("goal", self.goal_x)
                        print("current y", self.y)
                        print("goal", self.goal_y)
                       
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0 
                    
                    else:
                        twist.linear.x = 0.00
                        twist.angular.z = 0.0 
                        self.publisher_.publish(twist)
                        break                    
                    self.publisher_.publish(twist)
        finally:
            # stop moving   
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
    def run_combi(self,paths):
        for point in paths:
            self.travelling_point(point)
    def path(self):
        rclpy.spin_once(self)
        twist = geometry_msgs.msg.Twist()
        # table = 2
        # print("table",Table)
        self.table = int(input("input table number: "))
        try:
            if self.table == 1:
                
                while abs(int(self.orien*100)) >=2 :
                        print("Turning to table 1")
                        rclpy.spin_once(self)
                        print(math.degrees(self.orien))
                        print("dis", self.front)
                        twist.angular.z = 0.3
                        self.publisher_.publish(twist)
                print(self.front, "in 1st if")
                while self.front > 0.25:
                        rclpy.spin_once(self)
                        print("heading to table 1")
                        print(self.front)
                        twist.linear.x = 0.3
                        twist.angular.z = 0.0
                        self.publisher_.publish(twist)
                if self.front <= 0.25:
                    print("stopping")
                    print(self.front)
                    twist.linear.x =0.0
                    self.publisher_.publish(twist)
                    self.run_combi([0])

            # if self.table == 4 or 5:
            #     self.travelling_point(0)
            #     self.goal_y4 = waypoints[3][1][1]
            #     while ((int(abs(self.goal_y4)*100) -2 != int(abs(self.y)*100))) :
            #         rclpy.spin_once(self)
            #         print("moving")
            #         print("current y", self.y)
            #         print("goal", self.goal_y)
            #         twist.linear.x = 0.1
            #         twist.angular.z = 0.0    
            #         self.publisher_.publish(twist)  
            #         if (int(abs(self.goal_y4)*100) -3 == int(abs(self.y)*100)):
            #             twist.linear.x = 0.0
            #             twist.angular.z = 0.0 
            #             self.publisher_.publish(twist) 
            #             break

            if self.table in [2,3,4,5]:
                for points in paths[self.table]:
                    self.travelling_point(points)
                    # if self.table == 4 or 5:
                    #     print("in 4")
                    #     while (abs(int(abs(self.goal_y)*100)-int(abs(self.y)*100 ))>= 3):
                    #         rclpy.spin_once(self)
                    #         twist.linear.x = 0.1
                    #         twist.angular.z = 0.0 
                    #         self.publisher_.publish(twist)
                    #         if (abs(int(abs(self.goal_y)*100)-int(abs(self.y)*100 ))<= 2):
                    #             twist.linear.x = 0.0
                    #             twist.angular.z = 0.0 
                    #             self.publisher_.publish(twist)
                    #             break
                        
                if self.table == 3:
                    while abs(int(self.orien*100)) <=299 :
                        rclpy.spin_once(self)
                        twist.angular.z = 0.3
                        self.publisher_.publish(twist)
                        print("Turning to table")
                        print(math.degrees(self.orien))
                        print( abs(int(self.orien*100)))
                else:
                    while abs(int(self.orien*100)) >=2 :
                        print("Turning to table, not 3")
                        rclpy.spin_once(self)
                        print(math.degrees(self.orien))
                        twist.angular.z = 0.3
                        self.publisher_.publish(twist)
                while self.front > 0.3:
                    rclpy.spin_once(self)
                    print("moving to table")
                    twist.linear.x = 0.4
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    if self.front <= 0.2:
                        print("stopping at table")
                        twist.linear.x =0.0
                        re = 1
                new_path = paths[self.table][::-1]
                if self.table == (4 or 5):
                    new_path[-1] = 7
                print(new_path)
                self.run_combi(new_path)                    
            self.publisher_.publish(twist)   

            if self.table == 0:
                print("FAILED TO SUBSCIBE TO USER")
        finally:
            # stop moving   
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
def on_table_num(client, userdata, msg):
    global table_num 
    table_num = int(msg.payload.decode('utf-8'))
    print(table_num) # added cuz without this IT WONT WORK

    


def main(args = None):

    try:
        rclpy.init(args = args)
        auto_move = Auto_Mover()
        rclpy.spin_once(auto_move)  
        auto_move.path()
            # to get ip address of the laptop
        while True:
         #print (table_num)
        #  if(table_num != -1):
        #      # send message to esp32 to tell it that the robot has un-docked and is moving to the table
        #      client.publish("esp32/input", "0")
        #      #navigation.moveToTable(table_num)
        #      navigation.dock()
        #      table_num = -1
        #      # send message back to esp32 to tell it that the robot has docked
        #      client.publish("esp32/input", "1")
         
         pass
    except KeyboardInterrupt:
        auto_move.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
