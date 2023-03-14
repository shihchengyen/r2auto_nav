# storing way point into excel file
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
class Waypoint(Node):
    def __init__(self) -> None:
        super().__init__('waypoint')
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription

    def odom_callback(self,msg):        
        while True:
            
            # get keyboard input
            cmd_char = str(input("Press q to store point: "))
            if cmd_char == "q":
                pos = msg.pose.pose.position
                print(pos)
        

def main(args=None):
    rclpy.init(args=args)

    waypoint = Waypoint()
    rclpy.spin(waypoint)
    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
