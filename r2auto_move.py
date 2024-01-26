import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Bool
import numpy as np
import math
from math import atan2
import pickle
import cmath
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import time
MQTT_BROKER = '192.168.1.102'
# import tf2_ros
# from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
# import RPI.GPIO as GPIO
import paho.mqtt.client as mqtt
with open("waypoints_sim.pickle","rb") as handle:
    waypoints = pickle.load(handle)

print(waypoints)
mapfile = 'map.txt'
speedchange = 0.05
angle_error = 2
paths = {1:[0,1],2:[0,1,3],3:[0,1,2],4:[0,1,3,7],5:[0,4,8],6:[0,1,3,7,5,6]}
# print("in in in ")
count = 0
rotatechange = 0.1
table = 0
can = bool

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
    docked = ''
    dir = 0.0
    rot_q = 0.0
    orien = 5.0
    count = 0
    front = 5.0
    yaw = 0.0
    irdata = [-1,-1]
    isCanPresent = ''

    # range = np.array([])
    def __init__(self) -> None:
        self.x = -1
        self.y = -1
        super().__init__('auto_mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel',10)
        self.dock_publisher = self.create_publisher(Bool,'docking_status',5)
        # self.user_subscription = self.create_subscription(String,
                                                        #   'user',self.user_sub,10)
        # self.odom_subsription = self.create_subscription(Odometry,
        #     'odom',
        #     self.odom_callback,
        #     10)
        self.sim_can_subscription = self.create_subscription(Bool,'can',self.can_callback,10)
        # self.sim_dock_subscription = self.create_subscription(String,'dock',self.can_sub,10)
        # self.occ_subscription = self.create_subscription(
        #     OccupancyGrid,
        #     'map',
        #     self.odom_callback,
        #     qos_profile_sensor_data)
        # self.occ_subscription  # prevent unused variable warning
        # self.occdata = np.array([])

        self.map2base_subscription = self.create_subscription(Pose,'/map2base',self.odom_callback,10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)  
        self.IRLeft_subscriber = self.create_subscription(Int16,'IRLeft',self.ir_callbackL,3)
        self.IRRight_subscriber = self.create_subscription(Int16,'IRRight',self.ir_callbackR,3)
        
    def ir_callbackL(self, msg):
        self.irdata[0] = msg.data
    def ir_callbackR (self,msg):
        self.irdata[1] = msg.data
    def can_callback(self,msg):
        global can
        can = msg.data
    def odom_callback(self, msg):
        # # print("callback")
        #For map2base

        self.x = msg.position.x
        self.y = msg.position.y
        self.rot_q = msg.orientation
        # print("In callback")
        # print("self.orien",self.orien)
        #for odom

        # self.rot_q = msg.pose.pose.orientation
        # self.x = msg.pose.pose.position.x
        # self.y = msg.pose.pose.position.y

        self.orien = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
        
        # print(self.rot_q)
        
    def scan_callback(self, msg):
        # create numpy array
        # print("in scan callback")
        laser_range = np.array(msg.ranges)
        # print("break here:1")
        positive_range = laser_range[-30:-1]
        p = laser_range[-10:-1]
        n = laser_range[0:10]
        check_range = np.append(p,n)

        # print(laser_range[0])
        # taken_range = np.add(taken_range, laser_range[0:16])
        other_range = (laser_range[0:30])
        taken_range = np.append(other_range , positive_range)
        taken_range[taken_range==0] = np.nan
        laser_range[laser_range==0] = np.nan
        # print(laser_range)
        # taken_range = list(filter(lambda x: x == 0.0,taken_range))
        # find index with minimum value
        # self.front = laser_range[0]
        # print("break here:2")
        if np.isnan(taken_range).all() == True:
            self.front = 3.0
            # print("all NAN")
        else:
            lr2i = np.nanargmin(taken_range)
            self.front = taken_range[lr2i]
            # print("updated dist", self.front)
        if laser_range.size != 0:
            # print(taken_range)
            # use nanargmax as there are nan's in laser_range added to replace 0's
            self.dir = np.nanargmin(laser_range)
            self.check = laser_range[self.dir]
            # self.get_logger().info('Picked direction: %d %f m' % (self.dir,taken_range[self.dir]))
            return
        else:
            # self.dir = 0
            self.get_logger().info('No data!')
            # print("laser range",taken_range)
        
        
        # self.angle_go = math.radians(lr2i)
        # log the info
        # self.get_logger().info('Shortest distance at %i degrees' % lr2i)    
    def stopbot(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
    def rotatebot (self,angle):
        theta = angle
        twist = geometry_msgs.msg.Twist()
        degree_to_turn = theta - int(100*self.orien )
        while abs(degree_to_turn) > angle_error:
                degree_to_turn = theta - int(100*self.orien )
                # print("angle finding")
                # self.rotatebot(degree_to_turn)
                sign = np.sign(degree_to_turn)
                twist.linear.x = 0.0
                twist.angular.z = 0.3 * sign
                # self.get_logger().info("Turning to table 1")
                rclpy.spin_once(self)
                # self.get_logger().info("My orientation is " + str(math.degrees(self.orien)))
                print(math.degrees(self.orien))               
                self.publisher_.publish(twist)
        self.stopbot()
        
    def dock(self):
        
        twist = geometry_msgs.msg.Twist()
        print("in IR_follow")
        follow = True
        extreme = True
        while (follow == True):
            rclpy.spin_once(self)
            if(self.irdata[0]==0 and self.irdata[1]==0): #Front
                twist.linear.x = -0.04
                twist.angular.z = 0.0
                print(twist.linear.x)
                self.publisher_.publish(twist)
                print("publishing")
                if (extreme == True and self.irdata[0]!=0 and self.irdata[1]!=0): #extreme case where bot is perpendicular, run once
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    time.sleep(1)
                    self.publisher_.publish(twist)
                while (self.irdata[0]!=0 and self.irdata[1]!=0):
                    rclpy.spin_once(self)
                    twist.linear.x = -0.04
                    time.sleep(0.1)
                    self.publisher_.publish(twist)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.05
                    time.sleep(0.1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                    twist.angular.z = 0.0
                    time.sleep(0.1)
                    self.publisher_.publish(twist)
                extreme = False
            elif (self.irdata[0]==0 and self.irdata[1]!=0): #Right (Clockwise)
                twist.linear.x = 0.0
                twist.angular.z = 0.05
                self.publisher_.publish(twist)
                extreme = False
            elif (self.irdata[0]!=0 and self.irdata[1]==0): #Left (Counter-Clockwise)
                twist.linear.x = 0.0
                twist.angular.z = -0.05
                self.publisher_.publish(twist)
                extreme = False
            else: #Dont move
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                follow = False

    def test_angle(self):
        
        twist = geometry_msgs.msg.Twist()
        while True:
            twist.angular.z = 0.5  
            print(int(100*self.orien))
            self.publisher_.publish(twist)
            rclpy.spin_once(self)

    def pick_direction(self):
        print("excuted pick_direction")
        twist = geometry_msgs.msg.Twist()
        degree_to_turn = (int(100*math.radians(self.dir))) - int(100*self.orien )
        # rclpy.spin_once(self)
        while True:
            print("in while loop in pick direction")
            rclpy.spin_once(self)
            print(self.check)
            print(degree_to_turn)
            if self.check >= 0.5:
                print('check',self.check)
                print('self',self.dir)
                print(degree_to_turn)
                twist.linear.x = 0.05
                twist.angular.z = 0.0
            # elif self.check <= 0.48:
            #     self.stopbot()
            elif abs(degree_to_turn) > angle_error:
                degree_to_turn = (int(100*math.radians(self.dir))) - int(100*self.orien )
                twist.linear.x = 0.0
                twist.angular.z = 0.3
                print('check',self.check)
                print('self',self.dir)
            elif self.front >= 0.3:
                twist.linear.x = 0.05
                twist.angular.z = 0.0
            else:
                self.stopbot()
                break
                # self.rotatebot(int(100*math.radians(self.dir)))
            self.publisher_.publish(twist)




    def travelling_point(self, point):
        print("tavelling to waypoint")
        # points_char = int(input("enter waypoint to travel: "))
        twist = geometry_msgs.msg.Twist()
        # print("qewagdsfnc")
        
        # rclpy.init_node("speed_controller")
        # r = rclpy.Rate(4)
        if point == 10:
            print("point",point)
            self.goal_x = waypoints[2][0][0]
            self.goal_y = waypoints[2][0][1]
        elif point == 11:
            self.goal_x = waypoints[4][0][0]
            self.goal_y = waypoints[4][0][1]
        else:
            self.goal_x = waypoints[point][0][0]
            self.goal_y = waypoints[point][0][1]
       
        theta = (atan2(self.goal_y-self.y,self.goal_x-self.x))
        inc_x = 10000000 
        degree_to_turn = math.degrees(theta - self.orien )
        sign = np.sign(degree_to_turn)
        # degree_to_turn = math.degrees(theta - self.orien )
        try:

                while inc_x != 0:
                    # print("while in loop")
                    rclpy.spin_once(self)
                    # print(self.orien)
                   
                    inc_x = self.goal_x - self.x
                    current_angle =  math.degrees(self.orien)
                    # print("x",self.x, "inc",inc_x)
                    inc_y = self.goal_y - self.y
                    # degree_to_turn = (int(100*math.radians(theta))) - int(100*self.orien )
                    # if abs(degree_to_turn) - 2 > angle_error:
                    #     twist.linear.x = 0.0
                    #     twist.angular.z = 0.0
                    #     self.publisher_.publish(twist)
                    if  abs(degree_to_turn) > angle_error:
                        degree_to_turn = math.degrees(theta - self.orien )
                        # print("angle finding")
                        # self.rotatebot(degree_to_turn)
                        twist.linear.x = 0.0
                        twist.angular.z = 0.5 * sign
                        print("degree to turn", degree_to_turn)
                        print("current angle", math.degrees(self.orien))
                        print("theta", math.degrees(theta))
                        # print(sign)
                        print(sign)
                        print(point)
                    
                    
                    elif point == 3  or point == 4:
                        print("for point 3 and 4, but point is:",point)
                        if  (abs(int(abs(self.goal_y)*100)-int(abs(self.y)*100 ))) >=10 :
                            twist.linear.x = 0.05
                            twist.angular.z = 0.0  
                        if  (abs(int(abs(self.goal_y)*100)-int(abs(self.y)*100 ))) >=2 :

                            twist.linear.x = 0.1
                            twist.angular.z = 0.0  
                            
                            print("current y", self.y)
                            print("goal", self.goal_y)  
                        else:
                            twist.linear.x = 0.00
                            twist.angular.z = 0.0 
                            self.publisher_.publish(twist)
                            break 
                    elif (abs(int(abs(self.goal_x)*100 )-int(abs(self.x)*100 )))<=2:
                        twist.linear.x = 0.00
                        twist.angular.z = 0.0 
                        self.publisher_.publish(twist)
                        break                    
                    elif (abs(int(abs(self.goal_x)*100 -int(abs(self.x)*100 )))) >= 10:
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0  
                    elif (abs(int(abs(self.goal_x)*100 -int(abs(self.x)*100 ))))>=2:
                        # and abs((int(abs(self.goal_y)*100-2)-int(abs(self.y)*100))<= 3)
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0  
                        print("moving")
                        print("current x", self.x)
                        print("goal", self.goal_x)
                        print("current y", self.y)
                        print("goal", self.goal_y)
                        print("point",point)
                       
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0 
                    
                                  
                    self.publisher_.publish(twist)
        finally:
            # stop moving   
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

    def run_combi(self,paths):
        for point in paths:
            self.travelling_point(point)

    def dock(self):
        twist = geometry_msgs.msg.Twist()
        print("in IR_follow")
        self.follow = True
        extreme = True
        while (self.follow == True):
            print(self.irdata)
            rclpy.spin_once(self)
            if(self.irdata[0]==0 and self.irdata[1]==0): #Front
                twist.linear.x = -0.05
                twist.angular.z = 0.0
                print(twist.linear.x)
                self.publisher_.publish(twist)
                print("publishing")
                if (extreme == True and self.irdata[0]!=0 and self.irdata[1]!=0): #extreme case where bot is perpendicular, run once
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                while (self.irdata[0]!=0 and self.irdata[1]!=0):
                    rclpy.spin_once(self)
                    twist.linear.x = -0.05
                    self.publisher_.publish(twist)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3
                    time.sleep(0.1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                    twist.angular.z = 0.0
                    time.sleep(0.1)
                    self.publisher_.publish(twist)
                extreme = False
            elif (self.irdata[0]==0 and self.irdata[1]!=0): #Right (Clockwise)
                twist.linear.x = 0.0
                twist.angular.z = -0.3
                self.publisher_.publish(twist)
                extreme = False
            elif (self.irdata[0]!=0 and self.irdata[1]==0): #Left (Counter-Clockwise)
                twist.linear.x = 0.0
                twist.angular.z = 0.3
                self.publisher_.publish(twist)
                extreme = False
            elif (self.irdata[0]!=0 and self.irdata[1]!=0): #Dont move
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                self.follow = False

    def path(self):
        # global table
        # table  = int(input("Input table number:") )
        try:
            print(can)
            if table == 0:
                    print("FAILED TO SUBSCIBE TO USER")

            rclpy.spin_once(self)
            twist = geometry_msgs.msg.Twist()
            
            table 
            print("table", table)
            # print("table",Table)
            # table = int(input("input table number: "))
        
            if table == 1:
                self.run_combi(paths[table])

                #self.get_logger().info('My orientation is'+ str(self.orien))
                # while abs(int(self.orien * 100)) >= 2:
                #         self.get_logger().info("Turning to table 1")
                #         rclpy.spin_once(self)
                #         # self.get_logger().info("My orientation is " + str(math.degrees(self.orien)))
                #         print(math.degrees(self.orien))
                #         print("dis", self.front)
                #         twist.angular.z = 0.3
                #         self.publisher_.publish(twist)
                # while self.front > 0.25:
                #         rclpy.spin_once(self)
                #         print("heading to table 1")
                #         print(self.front)
                #         print(math.degrees(self.orien))
                #         twist.linear.x = 0.1
                #         twist.angular.z = 0.0                 
                                    
                #         self.publisher_.publish(twist)

                # if self.front <= 0.25:
                #     print("stopping")
                #     print(self.front)
                #     twist.linear.x =0.0
                #     self.publisher_.publish(twist)
                #     self.run_combi([0])
            if table == 6:
                rclpy.spin_once(self)
                self.run_combi(paths[table])
                # while abs(int(self.orien*100)) - 110 <=angle_error:
                #         print("Turning to table 6")
                #         rclpy.spin_once(self)
                #         print(math.degrees(self.orien))
                #         print("dis", self.front)
                #         twist.angular.z = 0.3
                #         self.publisher_.publish(twist)
                print("self dir in 6",self.dir)
                # 
                self.pick_direction()

        

            if table in [2,3,4,5]:
                for points in paths[table]:
                    self.travelling_point(points)
                        
                # if table == 3 or table == 4:
                #     while abs(int(self.orien*100)) <=311 :
                #         rclpy.spin_once(self)
                #         twist.angular.z = 0.3
                #         self.publisher_.publish(twist)
                #         print("Turning to table")
                #         print(math.degrees(self.orien))
                #         print( abs(int(self.orien*100)))
                # else:
                #     while abs(int(self.orien*100)) >= 2:
                #         print("Turning to table, not 3")
                #         rclpy.spin_once(self)
                #         # print(math.degrees(self.orien))
                #         print(abs(int(self.orien*100)))
                #         twist.angular.z = 0.22
                #         self.publisher_.publish(twist)
                # while self.front > 0.25:
                #     rclpy.spin_once(self)
                #     print("moving to table")
                #     twist.linear.x = 0.1
                #     twist.angular.z = 0.0  
                #     self.publisher_.publish(twist)
                    
                #     if self.front <= 0.25:
                #         print("stopping at table")
                #         twist.linear.x =0.0
            if table in [1,2,5]:
                self.rotatebot(310)
            else:
                self.rotatebot(0)
            new_path = paths[table][::-1]
            # if table is 4:
            #     new_path[0] = 7
            if table == 5:
                # new_path[-1] = 10
                new_path[0] = 11
            print(new_path)
            # insert can value here
            if can == False:
                
                print("returning")
                self.run_combi(new_path)
                self.rotatebot(-50)
                #put docking function here   
                self.dock()  
            else:
                time.sleep(10)
                print("returning but no can reading")
                self.run_combi(new_path)
                self.rotatebot(-50)
                
                #put docking function here   
                self.dock()  
        except KeyboardInterrupt:
            self.get_logger().info('I stopped')
            self.stopbot()
        #finally:
        #    self.stopbot()
            # stop moving   
            

def table_num(client, userdata, message):
    print("received message: " ,str(message.payload.decode("utf-8")))
    global table
    table = int(message.payload.decode("utf-8"))
    print(table)

def on_connect(client, userdata, flags, rc):
    # This will be called once the client connects
    print(f"Connected with result code {rc}")
    # Subscribe here!
    client.subscribe("TableNo")  


def main(args = None):

    try:
        rclpy.init(args = args)
        auto_move = Auto_Mover()
        # auto_move.test_angle()
        # auto_move.rotatebot(90) 
        
            # to get ip address of the laptop
        #connect to esp32
        client = mqtt.Client("Turtlebot")
        client.on_connect = on_connect
        client.on_message = table_num
        client.username_pw_set("roger", "password")
        client.connect(MQTT_BROKER, 1883)
        client.loop_start()
        while True:
            
            client.subscribe("TableNo")
            rclpy.spin_once(auto_move)
            # auto_move.dock()
            print(auto_move.isCanPresent)
            if can == True:
                auto_move.path()
            else:
                time.sleep(10)
                auto_move.path()

    except KeyboardInterrupt:
        client.disconnect()
        auto_move.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
