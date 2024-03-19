import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math
import cmath

# constants
linear_speed = 0.1
angle_speed = 0.5


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
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        publisher_period = 0.1  # seconds
        self.publisherTimer = self.create_timer(publisher_period, self.publisher_callback_fsm)
        
        # publisher_callback_fsm is used as the fsm to control the robot also
        self.state = "idle"
        self.targetAngle = 0
        self.currentYaw = 0

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
        self.linear_subscription = self.create_subscription(
            UInt8,
            'cmd_linear',
            self.linear_callback,
            10)

        ''' ================================================ cmd_angle ================================================ '''
        # Create a subscriber to the topic "cmd_angle"
        self.angle_subscription = self.create_subscription(
            Float64,
            'cmd_angle',
            self.angle_callback,
            10)

        self.get_logger().info("robotControlNode has started! :D")

    def odom_callback(self, msg):
        orientation_quat = msg.pose.pose.orientation
        self.yaw = angle_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def linear_callback(self, msg):
        if msg.data == 0:
            self.state = "idle"
            self.get_logger().info("linear_callback: idle")
        elif msg.data == 1:
            self.state = "forward"
            self.get_logger().info("linear_callback: forward")
        else:
            self.get_logger().info("Error: Publication to 'cmd_vel' must be 0 or 1, but got %d" % msg.data)

    def angle_callback(self, msg): 
        # takes in delta angle in degrees
        # +- 180 cannot work as the direction may change between initial and first iteration, +- 179 seems to work
        # +ve is anti clockwise, -ve is clockwise
        
        if msg.data == 0:
            self.state = "idle"
            self.get_logger().info("angle_callback: idle")
        else:
            self.state = "rotate"
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
            
            self.get_logger().info('angle_callback: complexChange: %s' % str(self.complexChange))
            self.get_logger().info('angle_callback: initialDirection: %f' % self.initialDirection)
            self.get_logger().info('angle_callback: Current: %f' % math.degrees(self.currentYaw))
            self.get_logger().info('angle_callback: Desired: %f' % math.degrees(cmath.phase(self.complexTargetYaw)))
            
    def move_forward(self):
        twist = Twist()

        twist.linear.x = linear_speed
        twist.angular.z = 0.0

        self.publisher_.publish(twist)

    def stop(self):
        twist = Twist()

        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.publisher_.publish(twist)

    def rotatebot(self):
        twist = Twist()
        
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        
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
            twist.angular.z = self.directionChange * angle_speed * speedScaling
        else:
            self.get_logger().info('End Yaw: %f' % math.degrees(self.currentYaw))
            # set the rotation speed to 0
            twist.angular.z = 0.0
            
            # set state back to idle
            self.state = "idle"

        # publish the twist message with angular speed
        self.publisher_.publish(twist)      
        
    def publisher_callback_fsm(self):
        if self.state == "idle":
            self.stop()
        elif self.state == "forward":
            self.move_forward()
        elif self.state == "rotate":
            self.rotatebot()
        else:
            self.state == "idle"

def main(args=None):
    rclpy.init(args=args)

    robot_control_node = RobotControlNode()
   
    try:
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_control_node.destroy_node()

if __name__ == '__main__':
    main()
