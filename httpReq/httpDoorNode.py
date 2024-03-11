import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .httpESP import HttpESP


class DoorHandler(Node):
    def __init__(self):
        super().__init__('doorRequestNode')
        
        # IP is hardcoded (for now?)
        self.espIP = "192.168.43.73"
        self.httpESP = HttpESP(self.espIP)
        
        # Create a subscriber to the topic "doorRequest"
        # Listers for the door request from the main control node
        self.subscription = self.create_subscription(
            String,
            'doorRequest',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create a publisher to the topic "doorStatus"
        # Publishes the status of the door to the main control node
        self.publisher_ = self.create_publisher(String, 'doorStatus', 10)
        publisher_period = 0.5  # seconds
        self.publisherTimer = self.create_timer(publisher_period, self.publisher_callback)
        self.statsToPublish = "idle"
        
        # for FSM
        self.state = "idle"
        fsm_period = 0.1  # seconds
        self.fsmTimer = self.create_timer(fsm_period, self.httpFSM)
        
        self.get_logger().info("httpdoornode has started! :D")

    def listener_callback(self, msg):
        # if the message is "openDoor" and the state is idle, then change the state to sendRequest
        if msg.data == "openDoor" and self.state == "idle":
            self.state = "sendRequest"
            self.get_logger().info("Requesting to open door")
    
    def publisher_callback(self):
        msg = String()
        
        # if ESP is online publish intended door status, else publish connection error
        if self.httpESP.pingESP():
            msg.data = self.statsToPublish
        else:
            msg.data = "connection error"
        
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing door status: %s" % msg.data)
        
    def httpFSM(self):
        if self.state == "idle":
            self.statsToPublish = "idle"
            pass
        elif self.state == "sendRequest":
            # send a request to the ESP, this is blocking so will return once its done, no need wait state
            # it blocks publishing also, idk i thought call back supposed to not be blocked??
            # there is a connection fail timeout so the try block to prevent crashing
            # ping first so we dont waste time if the ESP is offline
            if self.httpESP.pingESP():
                try:
                    self.reponse = self.httpESP.request()
                    
                    if self.reponse.status_code == 200:
                        # 200 = Success
                        # parse into dict
                        self.reponse = self.reponse.json()
                        
                        # get door number from the response data message
                        self.statsToPublish = self.reponse["data"]["message"]
                        
                        # print the door number
                        self.get_logger().info('Door Opened: "%s"' % self.statsToPublish)
                        self.state = "doorOpened"
                    else:
                        # 400 = Bad Request
                        # 500 = Internal Server Error
                        self.get_logger().info("http error")
                        self.state = "idle"
                except:
                    self.get_logger().info("connection failed :(")
                    self.statsToPublish = "connection error"
                    self.state = "idle"
            else:
                self.get_logger().info("ESP is offline")
                self.statsToPublish = "connection error"
                self.state = "idle"
            pass
        elif self.state == "doorOpened":
            # wait for the door to close???, ESP is not coded to do that lmao
            # since we dont receve any message from the ESP, fsm will be stuck here after opening door, 
            # reseting the ESP will not resolve also, so need to reset node
            pass
        else:
            self.state == "idle"
            pass

def main(args=None):
    rclpy.init(args=args)

    door = DoorHandler()

    try:
        rclpy.spin(door)
    except KeyboardInterrupt:
        pass
    finally:
        door.destroy_node()

if __name__ == '__main__':
    main()
