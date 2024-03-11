import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MainEG2310(Node):
    def __init__(self):
        super().__init__('mainNode')
        
        ''' http request -------------------------------------------------------------------------------------------'''
        # Create a subscriber to the topic "doorStatus"
        # Listens for the doorStatus from the doorRequestNode
        self.http_subscription = self.create_subscription(
            String,
            'doorStatus',
            self.http_listener_callback,
            10)
        self.http_subscription  # prevent unused variable warning
        
        # Create a publisher to the topic "doorRequest"
        # Publishes the door opening request to the doorRequestNode
        self.http_publisher = self.create_publisher(String, 'doorRequest', 10)
        
        # variable to be used to store the doorStatus
        self.doorStatus = ""
                
        ''' limit switch -------------------------------------------------------------------------------------------'''
        # Create a subscriber to the topic "switchStatus"
        # Listens for the switchStatus from the limitSwitchNode
        self.switch_subscription = self.create_subscription(
            String,
            'switchStatus',
            self.switch_listener_callback,
            10)
        self.switch_subscription  # prevent unused variable warning
        
        # Create a publisher to the topic "switchRequest"
        # Publishes the activate/deacivate request to the limitSwitchNode
        self.switch_publisher = self.create_publisher(String, 'switchRequest', 10)
        
        # variable to be used to store the limit switch status
        self.switchStatus = ""
        
        ''' servo control ------------------------------------------------------------------------------------------'''
        
        
        
        # Create a publisher to the topic "switchStatus"
        # Publishes the status of the limit switch to the main control node
        self.publisher_ = self.create_publisher(String, 'switchStatus', 10)
        publisher_period = 0.1  # seconds
        self.publisherTimer = self.create_timer(publisher_period, self.publisher_callback)
        
        # for FSM
        self.state = "idle"
        
        # for GPIO input
        GPIO.setmode(GPIO.BCM)
        self.sw_pin = 21
        GPIO.setup(self.sw_pin, GPIO.IN)
        
        self.get_logger().info("limitswitchnode has started! :D")

    def listener_callback(self, msg):
        # if the message is "activate" and the state is idle, then change the state to active
        if msg.data == "activate" and self.state == "idle":
            self.state = "active"
            self.get_logger().info("limitswitchnode publish has been activated")
        elif msg.data == "deactivate" and self.state == "active":
            self.state = "idle"
            self.get_logger().info("limitswitchnode publish has been deactivated")
            
    def switch_listener_callback(self, msg):
        # "released" or "pressed"
        self.switchStatus = msg.data
    
    def publisher_callback(self):
        if self.state == "active":
            msg = String()
            
            # GPIO.input returns 0 if pressed, 1 if released
            if GPIO.input(self.sw_pin):
                msg.data = "released"
            else:
                msg.data = "pressed"
            
            self.publisher_.publish(msg)
            self.get_logger().info("Publishing switchStatus: %s" % msg.data)
        

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