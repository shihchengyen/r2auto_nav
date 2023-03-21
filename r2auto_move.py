import math
import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import numpy as np
from math import atan2
import pickle
with open("waypoints.pickle","rb") as handle:
    waypoints = pickle.load(handle)
# print(waypoints)
# print("in in in ")
rotatechange = 0.1
speedchange = 0.05
x = 0.0
y = 0.0 
rot_q = 0.0 
theta = 0.0

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
        
        
        global theta
        twist = geometry_msgs.msg.Twist()
        points_char = int(input("enter waypoint to travel: "))
    
        # print("qewagdsfnc")
        
        # rclpy.init_node("speed_controller")
        # r = rclpy.Rate(4)
        goal_x = waypoints[points_char][0][0]
        goal_y = waypoints[points_char][0][1]
        inc_x = 10000000 
        try:

            while inc_x != 0:
                self.rot_q = msg.pose.pose.orientation
                angle_to_goal = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
                self.x = msg.pose.pose.position.x
                self.y = msg.pose.pose.position.y
                inc_x = goal_x - self.x
                # print("x",self.x, "inc",inc_x)
                inc_y = goal_y - y

                theta = atan2(inc_y,inc_x)

                if abs(angle_to_goal - theta) > 0.1:
                    print("this one",angle_to_goal)
                    print("angular",self.rot_q)
                    # print("theta" ,theta)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3
                else:
                    twist.linear.x = 0.5
                    twist.angular.z = 0.0 
                self.publisher_.publish(twist)
        finally:
            # stop moving   
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

        


    # def calling_point(self):
        # global theta
        # twist = geometry_msgs.msg.Twist()
        # points_char = int(input("enter waypoint to travel: "))
    
        # print("qewagdsfnc")
        # theta = euler_from_quaternion(self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w)
        # # rclpy.init_node("speed_controller")
        # # r = rclpy.Rate(4)
        # goal_x = waypoints[points_char][0][0]
        # goal_y = waypoints[points_char][0][1]
        # inc_x = 10000000 
        # try:

        #     while inc_x != 0:
        #         inc_x = goal_x - self.x
        #         print(self.x, "inc",inc_x)
        #         inc_y = goal_y - y

        #         angle_to_goal = atan2(inc_y,inc_x)

        #         if abs(angle_to_goal - theta) > 0.1:
        #             print(angle_to_goal)
        #             print(x,theta)
        #             twist.linear.x = 0.0
        #             twist.angular.z = 0.1
        #         else:
        #             twist.linear.x = 0.5
        #             twist.angular.z = 0.0 
        #         self.publisher_.publish(twist)
        # finally:
        #     # stop moving   
        #     twist.linear.x = 0.0
        #     twist.angular.z = 0.0
        #     self.publisher_.publish(twist)


def main(args = None):
    try:
        rclpy.init(args = args)
        auto_move = Auto_Mover()
        rclpy.spin_once(auto_move)  
        auto_move.calling_point()
    except KeyboardInterrupt:
        auto_move.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


