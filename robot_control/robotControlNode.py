import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float64, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math
import cmath

# constants
max_linear_speed = 0.1
max_angle_speed = 0.5  # anglularVel is in __???


# return the rotation angle around z axis in radians (counterclockwise)
def angle_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

# This node provides simplified API to control the robot.
# Other nodes can control the robot by publishing to the topic 'cmd_linear' and 'cmd_angle'.
# Usage:
#   - cmd_linear(UInt8): Send 0 to stop the robot, 1 to move the robot forward. Other values are considered as incorrect commands.
#   - cmd_angle(Float64): Send angle to rotate by degrees (counterclockwise). Negative values are also accepted.
#
# Important Note:
#   Publication to 'cmd_angle' stops linear motion commanded by 'cmd_linear'.
#   Likewise, publication to 'cmd_linear' stops angular motion commanded by 'cmd_angle'.
#   You should sleep for seconds after publishing to 'cmd_angle' to ensure the robot rotates by designated degrees.

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robotControlNode')

        ''' ================================================ TurtleBot control ================================================ '''
        # Create a publisher for moving TurtleBot
        self.publisherFSM = self.create_publisher(Twist, 'cmd_vel', 10)
        publisher_period = 0.1  # seconds
        self.publisherFSMTimer = self.create_timer(publisher_period, self.publisher_callback_fsm)
        
        # publisher_callback_fsm is used as the fsm to control the robot also
        self.state = "rotateStop"
        self.linearVel = 0.0
        self.angleVel = 0.0
        self.targetAngle = 0
        self.currentYaw = 0
        
        ''' ================================================ robotControlNode_state_feedback ================================================ '''
        # Create a publisher for robotControlNode_state_feedback
        self.publisherFB = self.create_publisher(String, 'robotControlNode_state_feedback', 10)
        publisher_period = 0.1  # seconds
        self.publisherFBTimer = self.create_timer(publisher_period, self.publisher_callback_state_feedback)

        ''' ================================================ odometry ================================================ '''
        # Create a subscriber to the topic "odom"
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning
        self.yaw = 0

        ''' ================================================ cmd_linear ================================================ '''
        # Create a subscriber to the topic "cmd_linear"
        # -1 to -127    ==> 0% to -100% of the maximum speed
        # 0             ==> stop
        # 1 to 127      ==> 0% to 100% of the maximum speed
        self.linear_subscription = self.create_subscription(
            Int8,
            'cmd_linear',
            self.linear_callback,
            10)

        ''' ================================================ cmd_anglularVel ================================================ '''
        # Create a subscriber to the topic "cmd_anglularVel"
        # -1 to -127    ==> 0% to -100% of the maximum speed, -ve is clockwise
        # 0             ==> stop
        # 1 to 127      ==> 0% to 100% of the maximum speed, +ve is anti clockwise
        self.angle_subscription = self.create_subscription(
            Int8,
            'cmd_anglularVel',
            self.anglularVel_callback,
            10)
        
        ''' ================================================ cmd_deltaAngle ================================================ '''
        # Create a subscriber to the topic "cmd_deltaAngle"
        # deltaAngle is in degrees, the angle to rotate by
        # +ve is anti clockwise, -ve is clockwise
        self.angle_subscription = self.create_subscription(
            Float64,
            'cmd_deltaAngle',
            self.deltaAngle_callback,
            10)

        self.get_logger().info("robotControlNode has started! :D")

    def odom_callback(self, msg):
        orientation_quat = msg.pose.pose.orientation
        self.yaw = angle_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def linear_callback(self, msg):
        # -1 to -127    ==> 0% to -100% of the maximum speed
        # 0             ==> stop
        # 1 to 127      ==> 0% to 100% of the maximum speed
        if msg.data == 0:
            self.linearVel = 0.0
            self.get_logger().info("linear_callback: linear_stop")
        else:
            self.linearVel = (msg.data / 127.0) * max_linear_speed
            self.get_logger().info("linear_callback: with msg %d and speed: %f" % (msg.data, self.linearVel))  
            
    def anglularVel_callback(self, msg):
        # -1 to -127    ==> 0% to -100% of the maximum speed, -ve is clockwise
        # 0             ==> stop
        # 1 to 127      ==> 0% to 100% of the maximum speed, +ve is anti clockwise
        if msg.data == 0:
            self.state = "rotateStop"
            self.get_logger().info("anglularVel_callback: rotateStop")
        else:
            self.state = "rotateWithVel"
            self.angleVel = (msg.data / 127.0) * max_angle_speed
            self.get_logger().info("anglularVel_callback: rotateWithVel with %d and speed: %f" % (msg.data, self.angleVel))
            
            # discard anything from deltaAngle_callback
            self.targetYaw = self.currentYaw

    def deltaAngle_callback(self, msg): 
        # Create a subscriber to the topic "cmd_deltaAngle"
        # deltaAngle is in degrees, the angle to rotate by
        # +ve is anti clockwise, -ve is clockwise
        # +- 180 cannot work as the direction may change between initial and first iteration, +- 179 seems to work
        
        if msg.data == 0:
            self.state = "rotateStop"
            self.get_logger().info("deltaAngle_callback: rotateStop")
        else:
            self.state = "rotateByAngle"
            self.currentYaw = self.yaw
            self.targetYaw = self.currentYaw + math.radians(msg.data)
            
            # we are going to use complex numbers to avoid problems when the angles go from
            # 360 to 0, or from -180 to 180
            self.complexCurrentYaw = complex(math.cos(self.currentYaw),math.sin(self.currentYaw))
            self.complexTargetYaw = complex(math.cos(self.targetYaw),math.sin(self.targetYaw))
            
            # divide the two complex numbers to get the change in direction
            self.complexChange = self.complexTargetYaw / self.complexCurrentYaw
            
            # get the sign of the imaginary component to figure out which way we have to turn            
            self.initialDirection = np.sign(cmath.phase(self.complexChange)) 
            
            self.get_logger().info('deltaAngle_callback: complexChange: %s' % str(self.complexChange))
            self.get_logger().info('deltaAngle_callback: initialDirection: %f' % self.initialDirection)
            self.get_logger().info('deltaAngle_callback: Current: %f' % math.degrees(self.currentYaw))
            self.get_logger().info('deltaAngle_callback: Desired: %f' % math.degrees(cmath.phase(self.complexTargetYaw)))
            
            # discard anything from anglularVel_callback
            self.angleVel = 0.0
            
    def rotatebot(self):
        if self.state == "rotateStop":
            return 0.0
        elif self.state == "rotateWithVel":
            return self.angleVel
        elif self.state == "rotateByAngle":
            self.currentYaw = self.yaw
            self.complexCurrentYaw = complex(math.cos(self.currentYaw),math.sin(self.currentYaw))
            self.get_logger().info('angle_callback: Current: %f' % math.degrees(self.currentYaw))
            
            # divide the two complex numbers to get the change in direction
            self.complexChange = self.complexTargetYaw / self.complexCurrentYaw
            # self.get_logger().info('complexChange: %s' % str(self.complexChange))
            
            # get the sign of the imaginary component to figure out which way we have to turn
            self.directionChange = np.sign(cmath.phase(self.complexChange))
            self.get_logger().info('directionChange: %f' % self.directionChange)
                        
            # if the direction of change is different from intital direction, then we have to rotate enough
            if self.initialDirection * self.directionChange > 0:
                # # speed scaling based on how much we have to rotate
                # speedScaling = 1 if (abs(cmath.phase(self.complexChange)) > 10) else (abs(cmath.phase(self.complexChange)) / 10)
                speedScaling = 1
                # self.get_logger().info('speedScaling: %f' % speedScaling)
                # set the direction and speed to rotate
                return self.directionChange * max_angle_speed * speedScaling
            else:
                self.get_logger().info('End Yaw: %f' % math.degrees(self.currentYaw))
                # set state back to rotateStop
                self.state = "rotateStop"
                
                # return rotation speed to 0
                return 0.0 
        else:
            self.state = "rotateStop"
            return 0.0
        
    def publisher_callback_fsm(self):
        twist = Twist()
        
        # self.linearVel is set from linear call back
        twist.linear.x = self.linearVel
        
        # twist.angular.z is set from the 
        twist.angular.z = self.rotatebot()
            
        self.publisherFSM.publish(twist)  
    
    def publisher_callback_state_feedback(self):
        # only publish the rotate states, linear no feedback
        msg = String()
        msg.data = self.state
        self.publisherFB.publish(msg)
        
    def custom_destroy_node(self):
        twist = Twist()
        # stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        self.publisherFSM.publish(twist)  
        
        self.destroy_node()
            
def main(args=None):
    rclpy.init(args=args)

    robot_control_node = RobotControlNode()
   
    try:
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_control_node.custom_destroy_node()

if __name__ == '__main__':
    main()