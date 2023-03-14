# storing way point into excel file
import rclpy
from rclpy.node import Node
from nav_msg.msg import Odometry
class Waypoint(Node):
    def __init__(self) -> None:
        super().__init__('waypoint')
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription

    def odom_callback(self,msg):
        pos = msg.pose.pose.position
        self.get_logger().info('i got: "%s"' % pos)
def main(args=None):
    rclpy.init(args=args)

    waypoint = Waypoint()
    rclpy.spin(waypoint)
    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
