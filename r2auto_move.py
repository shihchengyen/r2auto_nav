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
with open("waypoints.pickle","rb") as handle:
    waypoints = pickle.load(handle)

print(waypoints)
mapfile = 'map.txt'

paths = {2:[2],3:[2],4:[3],5:[4]}
print("in in in ")
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
        print("callback")
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
        laser_range = np.array(msg.ranges)
        positive_range = laser_range[-15:-1]
        # print(laser_range[0])
        # taken_range = np.add(taken_range, laser_range[0:16])
        other_range = (laser_range[0:14])
        taken_range = np.append(other_range , positive_range)
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        # self.front = laser_range[0]
        lr2i = np.nanargmin(taken_range)
        self.front = taken_range[lr2i]
        self.angle_go = math.radians(lr2i)
        # log the info
        # self.get_logger().info('Shortest distance at %i degrees' % lr2i)    


    def travelling_point(self, point):
        # points_char = int(input("enter waypoint to travel: "))
        twist = geometry_msgs.msg.Twist()
        # print("qewagdsfnc")
        
        # rclpy.init_node("speed_controller")
        # r = rclpy.Rate(4)
        goal_x = waypoints[point][0][0]
        goal_y = waypoints[point][0][1]
        theta = atan2(goal_y-self.y,goal_x-self.x)
        inc_x = 10000000 
        try:

                while inc_x != 0:
                    # print("while in loop")
                    rclpy.spin_once(self)
                    # print(self.orien)
                   
                    
                    inc_x = goal_x - self.x

                    # print("x",self.x, "inc",inc_x)
                    inc_y = goal_y - self.y
                    if int(abs(goal_x)*100) == int(abs(self.x)*100):
                        print("stopping")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0 
                        break
                    elif abs(self.orien - theta) > 0.1:
                        print("angle finding")
                        # print(self.orien)
                        # print(self.x)
                        # print(int(abs(self.orien)*100))
                        # print(int(abs(theta)*100))
                        # # print("theta", theta)
                        # if (self.orien  and theta > 0) or (self.orien and theta < 0):
                        #     print("in 1")
                        #     if abs(self.orien) > abs(theta):
                        #         twist.linear.z = 0.3
                                
                        #     else: 
                        #         twist.linear.z = -0.3
                            
                        # elif self.orien - theta > self.orien:
                        #     print("in 2")
                        #     twist.angular.z = 0.3
                        #     twist.linear.x = 0.0
                        # else:
                        #     print("in 3")
                        twist.angular.z = 0.3
                        twist.linear.x = 0.0
                    elif goal_x != self.x:
                        # print("moving")
                        # print("current x", self.x)
                        # print("goal", goal_x)
                        # print("current y", self.y)
                        # print("goal", goal_y)
                        if abs(self.x) > abs(goal_x):
                            twist.linear.x = 0.05
                            twist.angular.z = 0.0
                        else:
                            twist.linear.x = -0.05
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
        twist = geometry_msgs.msg.Twist()
        if self.count == 0:
            Table = self.table
            self.count = 1
        # print("table",Table)
        # Table = int(input("input table number: "))
        try:
            if Table == 1:
                while self.front > 0.2:
                        twist.linear.x = 0.3
                        twist.angular.z = 0.0
                        self.publisher_.publish(twist)
                if self.front <= 0.2:
                    twist.linear.x =0.0

                
            if Table == 2 or 3 or 4 or 5 :
                for points in paths[Table]:
                    self.travelling_point(points)
                if Table == 3:
                    while abs(int(self.orien*100)) <=299 :
                        twist.angular.z = 0.3
                        self.publisher_.publish(twist)
                else:
                    while abs(int(self.orien*100)) >=2 :
                        twist.angular.z = 0.3
                        self.publisher_.publish(twist)
                while self.front > 0.2:
                    twist.linear.x = 0.3
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                if self.front <= 0.2:
                    twist.linear.x =0.0
                for point    in paths[::-1]:
                    self.travelling_point(point)

                    
                    
            self.publisher_.publish(twist)   

            if Table == 0:
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
        auto_move.path()
    except KeyboardInterrupt:
        auto_move.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
