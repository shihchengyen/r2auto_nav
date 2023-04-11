import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import numpy as np
import math
from math import atan2
import pickle
import cmath
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import time
tables=[]
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
paths = {1:[0],2:[0,2],3:[0,2],4:[0,2,3],5:[0,4],6:[0,2,5,6]}
# print("in in in ")
count = 0
rotatechange = 0.1

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
    dir = 0.0
    table = 0
    rot_q = 0.0
    orien = 5.0
    count = 0
    front = 5.0
    yaw = 0.0
    # range = np.array([])
    def __init__(self) -> None:
        
        self.x = -1
        self.y = -1
        super().__init__('auto_mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel',10)
        # self.user_subscription = self.create_subscription(String,
                                                        #   'user',self.user_sub,10)
        self.odom_subsription = self.create_subscription(Odometry,
            'odom',
            self.odom_callback,
            10)
        self.sim_can_subscription = self.create_subscription(String,'can',self.can_sub,10)
        self.sim_dock_subscription = self.create_subscription(String,'dock',self.can_sub,10)
        # self.occ_subscription = self.create_subscription(
        #     OccupancyGrid,
        #     'map',
        #     self.odom_callback,
        #     qos_profile_sensor_data)
        # self.occ_subscription  # prevent unused variable warning
        # self.occdata = np.array([])
        # self.map2base_subscription = self.create_subscription(Pose,'/map2base',self.odom_callback,10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)  
        
    def on_message(client, userdata, message):
        # tables.append(str(message.payload.decode("utf-8")))
        print("received message: " ,str(message.payload.decode("utf-8")))
        # print(tables)
        global table
        table = str(message.payload.decode("utf-8"))

    def on_connect(client, userdata, flags, rc):
        # This will be called once the client connects
        print(f"Connected with result code {rc}")
        # Subscribe here!
        client.subscribe("TableNo")

    def can_sub(self,msg):
        global can
        can = msg.data
        self.c = can
    def docked_sub(self,msg):
        global docked
        docked = msg.data
        


    client = mqtt.Client("Turtlebot")
    client.on_connect = on_connect
    client.on_message = on_message
    client.username_pw_set("roger", "password")
    client.connect(MQTT_BROKER, 1883)

    if docked == 'Y'or time.sleep(10):
    
        client.loop_start()
    if can == 'Y':
        client.loop_stop()

    




    def odom_callback(self, msg):
        # # print("callback")
        # try:
        #     trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        # except (LookupException, ConnectivityException, ExtrapolationException) as e:
        #     self.get_logger().info('No transformation found')
        #     return
        #For map2base
        # self.x = msg.position.x
        # self.y = msg.position.y
        # self.rot_q = msg.orientation
        # print("In callback")
        # print("self.orien",self.orien)
        #for map  
        # cur_pos = trans.transform.translation
        # cur_rot = trans.transform.rotation
        # self.x = cur_pos.x
        # self.y = cur_pos.y
        # self.orien = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        #for odom
        self.rot_q = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.orien = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
        
        # print(self.rot_q)
        
        # points_char = int(input("enter waypoint to travel: "))
    
        # # print("qewagdsfnc")
    
    # def user_sub(self, msg):
    #     self.table = int(msg.data)
        # print("in subcriber user")    

    def scan_callback(self, msg):
        # create numpy array
        # print("in scan callback")
        laser_range = np.array(msg.ranges)
        # print("break here:1")
        positive_range = laser_range[-30:-1]
        p = laser_range[-10:-1]
        n = laser_range[0:10]
        check_range = np.append(n,p)

        # print(laser_range[0])
        # taken_range = np.add(taken_range, laser_range[0:16])
        other_range = (laser_range[0:30])
        taken_range = np.append(other_range , positive_range)
        taken_range[taken_range==0] = np.nan
        # print(laser_range)
        # taken_range = list(filter(lambda x: x == 0.0,taken_range))
        # find index with minimum value
        # self.front = laser_range[0]
        # print("break here:2")
        if np.isnan(check_range).all() == True:
            self.front = 3.0
            # print("all NAN")
        else:
            lr2i = np.nanargmin(check_range)
            self.front = check_range[lr2i]
            # print("updated dist", self.front)
        if taken_range.size != 0:
            # print(taken_range)
            # use nanargmax as there are nan's in laser_range added to replace 0's
            self.dir = np.nanargmin(taken_range)
            # self.get_logger().info('Picked direction: %d %f m' % (self.dir,taken_range[self.dir]))
            return
        else:
            self.dir = 0
            self.get_logger().info('No data!')
            # print("laser range",taken_range)
        
        
        # self.angle_go = math.radians(lr2i)
        # log the info
        # self.get_logger().info('Shortest distance at %i degrees' % lr2i)    
    
    def pick_direction(self):
        try:
            rclpy.spin_once(self)
            
            twist = geometry_msgs.msg.Twist()
            # self.get_logger().info('In pick_direction')
            print("self.dir",self.dir)
            angle = self.dir
            degree_to_turn = angle - math.degrees(self.orien ) 
            print("degree to turn",(degree_to_turn))
            print("dtt ", angle)
            print("orien",math.degrees(self.orien))
            print("front",self.front)
            while self.front > 0.25:
                print("in loop")
                # self.rotatebot(degree_to_turn)
                rclpy.spin_once(self)
                if abs(degree_to_turn) > angle_error:
                    degree_to_turn = angle - math.degrees(self.orien ) 
                    print("degree to turn",(degree_to_turn))
                    print("dtt ", angle)
                    print("orien",math.degrees(self.orien))
                    # print(self.dir)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                
               
                else:      
                    print("heading to table 6")
                    print(self.front)
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0                 
                                    
                self.publisher_.publish(twist)
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0                 
                                    
            self.publisher_.publish(twist)



    def travelling_point(self, point):
        print("tavelling to waypoint")
        # points_char = int(input("enter waypoint to travel: "))
        twist = geometry_msgs.msg.Twist()
        # print("qewagdsfnc")
        
        # rclpy.init_node("speed_controller")
        # r = rclpy.Rate(4)
        if point == 7:
            print("point",point)
            self.goal_x = waypoints[0][0][0]
            self.goal_y = waypoints[0][0][1]
        elif point == 8:
            self.goal_x = waypoints[4][0][0]
            self.goal_y = waypoints[4][0][1]
        else:
            self.goal_x = waypoints[point][0][0]
            self.goal_y = waypoints[point][0][1]
       
        theta = (atan2(self.goal_y-self.y,self.goal_x-self.x))
        inc_x = 10000000 
        degree_to_turn = math.degrees(theta - self.orien )
        self.sign = np.sign(degree_to_turn)
        degree_to_turn = math.degrees(theta - self.orien )
        try:

                while inc_x != 0:
                    # print("while in loop")
                    rclpy.spin_once(self)
                    # print(self.orien)
                   
                    inc_x = self.goal_x - self.x
                    current_angle =  math.degrees(self.orien)
                    # print("x",self.x, "inc",inc_x)
                    inc_y = self.goal_y - self.y
                    if  abs(degree_to_turn) > angle_error:
                        degree_to_turn = math.degrees(theta - self.orien )
                        # print("angle finding")
                        # self.rotatebot(degree_to_turn)
                        twist.linear.x = 0.0
                        twist.angular.z = 0.5 * self.sign
                        print("degree to turn", degree_to_turn)
                        print("current angle", math.degrees(self.orien))
                        print("theta", math.degrees(theta))
                        print(self.sign)
                        print(point)
                        # print("current x", self.x)
                        # print("goal", self.goal_x)
                        # print("current y", self.y)
                        # print("goal", self.goal_y)
                    elif point == 3 or point == 7 or point == 4:
                        print("for point 3 and 4, but point is:",point)
                        if  (abs(int(abs(self.goal_y)*100)-int(abs(self.y)*100 ))) >=2 :
                            if self.goal_y > self.y:

                                twist.linear.x = 0.1
                                twist.angular.z = 0.0  
                            if self.goal_y < self.y :
                                twist.linear.x = -0.1
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
                    elif (abs(int(abs(self.goal_x)*100 -int(abs(self.x)*100 )))) >= 2:
                        # and abs((int(abs(self.goal_y)*100-2)-int(abs(self.y)*100))<= 3)
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
    def path(self):
        rclpy.spin_once(self)
        twist = geometry_msgs.msg.Twist()
        
        table = self.table
        print("table", table)
        # print("table",Table)
        # self.table = int(input("input table number: "))
        
        try:
            if self.table == 1:
                
                while abs(int(self.orien*100)) <=310:
                        print("Turning to table 1")
                        rclpy.spin_once(self)
                        print(math.degrees(self.orien))
                        print("dis", self.front)
                        twist.angular.z = 0.3
                        self.publisher_.publish(twist)
                while self.front > 0.25:
                        twist.linear.x = 0.0
                        rclpy.spin_once(self)
                        print("heading to table 1")
                        print(self.front)
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0                 
                                  
                        self.publisher_.publish(twist)

                if self.front <= 0.25:
                    print("stopping")
                    print(self.front)
                    twist.linear.x =0.0
                    self.publisher_.publish(twist)
                    self.run_combi([0])
            if self.table == 6:
                rclpy.spin_once(self)
                # self.run_combi(paths[self.table])
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

        

            if self.table in [2,3,4,5]:
                for points in paths[self.table]:
                    self.travelling_point(points)
                        
                if self.table == 3 or self.table == 4:
                    while abs(int(self.orien*100)) >=2 :
                        rclpy.spin_once(self)
                        twist.angular.z = 0.3
                        self.publisher_.publish(twist)
                        print("Turning to table")
                        print(math.degrees(self.orien))
                        print( abs(int(self.orien*100)))
                else:
                    while abs(int(self.orien*100)) <= 310:
                        print("Turning to table, not 3")
                        rclpy.spin_once(self)
                        # print(math.degrees(self.orien))
                        print(abs(int(self.orien*100)))
                        twist.angular.z = 0.3
                        self.publisher_.publish(twist)
                while self.front > 0.25:
                    rclpy.spin_once(self)
                    print("moving to table")
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0 
                    self.publisher_.publish(twist)
                    
                    if self.front <= 0.25:
                        print("stopping at table")
                        twist.linear.x =0.0
                        
            new_path = paths[self.table][::-1]
            if self.table in  [4 , 5]:
                new_path[-1] = 7
            if self.table == 5:
                new_path[0] = 8
            print(new_path)
            if self.c == 'N'or time.sleep(10):
                self.run_combi(new_path)  
                print("returning")
                #put docking function here                  
            

            if self.table == 0:
                print("FAILED TO SUBSCIBE TO USER")
        finally:
            # stop moving   
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
    
def main(args = None):

    try:
        rclpy.init(args = args)
        auto_move = Auto_Mover()
        rclpy.spin_once(auto_move) 
        # auto_move.rotatebot(90) 
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
