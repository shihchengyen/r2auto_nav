# simulated user input 
# publisher 

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'user', 10)
        timer_period = 0.5# seconds
        self.timer = self.create_timer(timer_period, self.user_publish)
        self.i = 0

    def user_publish(self):
        msg = String()
        msg.data =(input("Table number: "))
        while True:

            self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)
    
    minimal_publisher = MinimalPublisher()
   
    rclpy.spin(minimal_publisher)
    minimal_publisher.user_publish()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()