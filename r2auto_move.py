import math
import rclpy
from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.qos import qos_profile_sensor_data
# from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math

# import numpy as np
from math import atan2
import pickle
# from math import pi
from time import sleep
with open("waypoints.pickle","rb") as handle:
    waypoints = pickle.load(handle)
# print(waypoints)
# print("in in in ")
rotatechange = 0.1
speedchange = 0.05
x = 0.0
y = 0.0 

theta = 0.0
table = 0
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
    orien = -1.1
    def __init__(self):
        self.table = 0
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
        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     'scan',
        #     self.listener_callback,
        #     qos_profile_sensor_data)        
        
    def odom_callback(self, msg):
        # print("odom")
        self.rot_q = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.orien = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
    
    def user_sub(self, msg):
        # print("user")
        # print(msg)
        # self.table = 0
        self.table = int(msg.data)
        # print(self.table)
        # self.get_logger().info("table: %s" % msg.data)

    def listener_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        positive_range = laser_range[-15:-1]
        # taken_range = np.add(taken_range, laser_range[0:16])
        
        other_range = (laser_range[0:14])
        taken_range = np.append(other_range , positive_range)
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        lr2i = np.nanargmin(taken_range)
        self.dis = taken_range[lr2i]
        self.angle_go = math.radians(lr2i)
        # log the info
        # self.get_logger().info('Shortest distance at %i degrees' % lr2i)

    def travelling_point(self, point):
        twist = geometry_msgs.msg.Twist()
        goal_x = waypoints[point][0][0]
        goal_y = waypoints[point][0][1]
        theta = atan2(goal_y-y,goal_x-x)
        inc_x = 10000000 
        try:
                while inc_x != 0:
                    # print("while in loop")
                    rclpy.spin_once(self)
                    # print(self.orien)
                    # self.orien = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
                    
                    inc_x = goal_x - self.x

                    # print("x",self.x, "inc",inc_x)
                    # inc_y = goal_y - self.y
                    if int(abs(goal_x)*100)-2 == int(abs(self.x)*100):
                        print("stopping",point)
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0 
                        break
                    
                    elif goal_x != self.x:
                        if int(abs(self.x)*100) > int(abs(goal_x)*100):
                            twist.linear.x = -0.1
                            twist.angular.z = 0.0
                        else:
                            twist.linear.x = 0.1
                            twist.angular.z = 0.0
                    elif abs(self.orien - theta) > 0.1:
                        # print("angle finding")
                        # print(self.orien)
                        twist.angular.z = 0.3
                        twist.linear.x = 0.0

                    self.publisher_.publish(twist)
        finally:
            # stop moving   
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
       
       
    def run_combi(self,path): 
        twist = geometry_msgs.msg.Twist()
        for point in path:
                print("point",point)
                self.travelling_point(point) 
        sleep(2)
        for point in path[::-1]:
            #   print("point",point)
              self.travelling_point(point) 
   
    def path(self):
        Table = self.table
        # print("table:",self.table)
        # Table = int(input("table numer: "))
        if Table == 1:
            path = [2,3,1]
            self.run_combi(path)
            
        if Table == 2:
            path=[1,3,2]
            self.run_combi(path)
        


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


