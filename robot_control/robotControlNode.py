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
angle_speed = 0.1


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
        self.linear_subscription = self.create_subscription(
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
            self.stop()
        elif msg.data == 1:
            self.move_forward()
        else:
            self.get_logger("Error: Publication to 'cmd_vel' must be 0 or 1, but got %d" % msg.data)

    def angle_callback(self, msg):
        self.rotatebot(msg.data)

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

    def rotatebot(self, rot_angle):
        twist = Twist()

        # get current yaw angle
        current_yaw = self.yaw
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * angle_speed
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while (c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
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
