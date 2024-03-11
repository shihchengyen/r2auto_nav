import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MainEG2310(Node):
    def __init__(self):
        super().__init__('mainNode')
        
        ''' ================================================ http request ================================================ '''
        # Create a subscriber to the topic "doorStatus"
        # Listens for the doorStatus from the doorRequestNode
        self.http_subscription = self.create_subscription(
            String,
            'doorStatus',
            self.http_listener_callback,
            10)
        self.http_subscription  # prevent unused variable warning
        
        # variable to be used to store the doorStatus
        self.doorStatus = ""
        
        # Create a publisher to the topic "doorRequest"
        # Publishes the door opening request to the doorRequestNode
        self.http_publisher = self.create_publisher(String, 'doorRequest', 10)
                
        ''' ================================================ limit switch ================================================ '''
        # Create a subscriber to the topic "switchStatus"
        # Listens for the switchStatus from the limitSwitchNode
        self.switch_subscription = self.create_subscription(
            String,
            'switchStatus',
            self.switch_listener_callback,
            10)
        self.switch_subscription  # prevent unused variable warning
        
        # variable to be used to store the limit switch status
        self.switchStatus = ""
        
        # Create a publisher to the topic "switchRequest"
        # Publishes the activate/deacivate request to the limitSwitchNode
        self.switch_publisher = self.create_publisher(String, 'switchRequest', 10)        
        
        ''' ================================================ servo control ================================================ '''
        # Create a publisher to the topic "servoRequest"
        # Publishes the servoRequest to the servoControlNode
        self.publisher_ = self.create_publisher(String, 'servoRequest', 10)
        
        ''' ================================================ main FSM ================================================ '''
        self.state = "idle"
        
        self.get_logger().info("mainNode has started, bitchesss! >:D")

    def listener_callback(self, msg):
        # if the message is "activate" and the state is idle, then change the state to active
        if msg.data == "activate" and self.state == "idle":
            self.state = "active"
            self.get_logger().info("limitswitchnode publish has been activated")
        elif msg.data == "deactivate" and self.state == "active":
            self.state = "idle"
            self.get_logger().info("limitswitchnode publish has been deactivated")
            
    def http_listener_callback(self, msg):
        # "idle", "door1", "door2", "connection error", "http error"
        self.doorStatus = msg.data
            
    def switch_listener_callback(self, msg):
        # "released" or "pressed"
        self.switchStatus = msg.data

def main(args=None):
    rclpy.init(args=args)

    mainEG2310 = MainEG2310()

    try:
        rclpy.spin(mainEG2310)
    except KeyboardInterrupt:
        pass
    finally:
        mainEG2310.destroy_node()

if __name__ == '__main__':
    main()