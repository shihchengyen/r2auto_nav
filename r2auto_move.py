import math
import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import numpy as np
from math import atan2
import pickle
from math import pi
with open("waypoints.pickle","rb") as handle:
    waypoints = pickle.load(handle)
print(waypoints)
# print("in in in ")
rotatechange = 0.1
speedchange = 0.05
x = 0.0
y = 0.0 
rot_q = 0.0 
theta = 0.0
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
    def __init__(self) -> None:
        self.x = -1
        self.y = -1
        super().__init__('auto_mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel',10)
        self.odom_subsription = self.create_subscription(Odometry,
            'odom',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        self.rot_q = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.orien = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
        # print("callback")
        # print(self.rot_q)
        
        # points_char = int(input("enter waypoint to travel: "))
    
        # # print("qewagdsfnc")
        
        

        


    def travelling_point(self, point):
        # points_char = int(input("enter waypoint to travel: "))
        twist = geometry_msgs.msg.Twist()
        # print("qewagdsfnc")
        
        # rclpy.init_node("speed_controller")
        # r = rclpy.Rate(4)
        goal_x = waypoints[point][0][0]
        goal_y = waypoints[point][0][1]
        theta = atan2(goal_y-y,goal_x-x)
        inc_x = 10000000 
        try:

                while inc_x != 0:
                    # print("while in loop")
                    rclpy.spin_once(self)
                    # print(self.orien)
                    self.orien = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
                    
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
                        print(self.orien)
                        # print("theta", theta)
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
                        print("moving")
                        print(self.orien)
                        print("current x", self.x)
                        print("goal", goal_x)
                        # print("current y", self.y)
                        # print("goal", goal_y)
                        if abs(self.x) > abs(goal_x):
                            twist.linear.x = -0.05
                            twist.angular.z = 0.0
                        else:
                            twist.linear.x = 0.05
                            twist.angular.z = 0.0

                    
                    self.publisher_.publish(twist)
        finally:
            # stop moving   
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
       
       
    def run_combi(self,path): 
        for point in path:
                print(path)
                self.travelling_point(point)   
    def path(self):
        Table = int(input('table number:'))
        if Table == 1:
            path = (1,2,3)
            self.run_combi(path)
            
        if Table == 2:
            path=(2,1,3)
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


