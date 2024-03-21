import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8, UInt16, Float64, String, Int8
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math


# return the rotation angle around z axis in degrees (counterclockwise)
def angle_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(t3, t4))


class MasterNode(Node):
    def __init__(self):
        super().__init__('masterNode')

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
        self.servo_publisher = self.create_publisher(String, 'servoRequest', 10)

        ''' ================================================ lidar ================================================ '''
        # Create a subscriber to the topic "scan"
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        
        ''' ================================================ bucket ================================================ '''
        # Listens for the bucket angle
        self.bucketAngle_subscription = self.create_subscription(
            UInt16,
            'bucketAngle',
            self.bucketAngle_listener_callback,
            10)
        self.bucketAngle_subscription  # prevent unused variable warning  

        ''' ================================================ occupancy map ================================================ '''
        # Create a subscriber to the topic "map"
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        self.yaw = 0

        ''' ================================================ robot position ================================================ '''
        # Create a subscriber to the topic
        self.pos_subscription = self.create_subscription(
            Pose,
            'position',
            self.pos_callback,
            10)

        ''' ================================================ cmd_linear ================================================ '''
        # Create a publisher to the topic "cmd_linear", which can stop and move forward the robot
        self.linear_publisher = self.create_publisher(Int8, 'cmd_linear', 10)

        ''' ================================================ cmd_anglularVel ================================================ '''
        # Create a publisher to the topic "cmd_angle", which can rotate the robot
        self.anglularVel_publisher = self.create_publisher(Int8, 'cmd_anglularVel', 10)
        
        ''' ================================================ cmd_deltaAngle ================================================ '''
        # Create a publisher to the topic "cmd_angle", which can rotate the robot
        self.deltaAngle_publisher = self.create_publisher(Float64, 'cmd_deltaAngle', 10)
        
        ''' ================================================ robotControlNode_state_feedback ================================================ '''
        # Create a subscriber to the robotControlNode_state_feedback
        self.pos_subscription = self.create_subscription(
            String,
            'robotControlNode_state_feedback',
            self.robotControlNode_state_feedback_callback,
            10)
        
        ''' ================================================ Master FSM ================================================ '''
        self.state = "idle"
        fsm_period = 0.1  # seconds
        self.fsmTimer = self.create_timer(fsm_period, self.masterFSM)
    
        self.closestAngle = 0
        
        # Create a subscriber to the topic fsmDebug
        # to inject state changes for debugging in RQT
        self.pos_subscription = self.create_subscription(
            String,
            'fsmDebug',
            self.fsmDebug_callback,
            10)

        self.get_logger().info("MasterNode has started, bitchesss! >:D")

    def http_listener_callback(self, msg):
        # "idle", "door1", "door2", "connection error", "http error"
        self.doorStatus = msg.data

    def switch_listener_callback(self, msg):
        # "released" or "pressed"
        self.switchStatus = msg.data

    def scan_callback(self, msg):
        # create numpy array to store lidar data
        self.laser_range = np.array(msg.ranges)
        
        # read min and max range values
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        
        # replace out of range values with nan
        self.laser_range[self.laser_range < self.range_min] = np.nan
        self.laser_range[self.laser_range > self.range_max] = np.nan
        
        # store the len since it changes
        self.range_len = len(self.laser_range)
        
    def bucketAngle_listener_callback(self, msg):
        self.bucketAngle = msg.data

    def occ_callback(self, msg):
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata, occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)
        self.map_res = msg.info.resolution  # according to experiment, should be 0.05 m

    def pos_callback(self, msg):
        # Note: those values are different from the values obtained from odom
        self.pos_x = msg.position.x
        self.pos_y = msg.position.y
        # in degrees (not radians)
        self.yaw = angle_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.get_logger().info('x y yaw: %f %f %f' % (self.pos_x, self.pos_y, self.yaw))
        
    def robotControlNode_state_feedback_callback(self, msg):
        self.robotControlNodeState = msg.data
        
    def fsmDebug_callback(self, msg):
        self.state = msg.data
        
    def index_to_angle(self, index, arrLen):
        # return in degrees
        return (index / (arrLen - 1)) * 359
    
    def masterFSM(self):
        if self.state == "idle":
            pass
        elif self.state == "checking_walls_distance":
            # lidar minimum is 12 cm send by node, datasheet says 16 cm
            # by experimentation need 30 cm
            # if less than 30 cm from nearest object, move away from it, else can find the bucket using bucketFinderNode
            # bucket finder doesnt work if its too close to wall
            min_distance = min(self.laser_range)
            
            self.get_logger().info('min_distance %f' % min_distance)
            
            if min_distance < 0.3:
                self.get_logger().info('too close! moving away')
                
                # get angle
                argmin = np.nanargmin(self.laser_range)
                angle_min = self.index_to_angle(argmin, self.range_len)
                
                self.get_logger().info('angle_min %f' % angle_min)
                
                # set linear to be zero 
                linear_msg = UInt8()
                linear_msg.data = 0
                self.linear_publisher.publish(linear_msg)
                
                # angle_min > or < 180, the delta angle to move away from the object is still the same
                deltaAngle_msg = Float64()
                deltaAngle_msg.data = angle_min - 180.0
                self.angle_publisher.publish(deltaAngle_msg)

                self.state = "rotating_to_move_away_from_walls"

            else:
                self.state = "rotating_to_bucket"
                
                
                
                # if the closest object is in not at the back of the robot, rotate first, else move away from it
                if abs(angle_min - 180) > 5:
                    # set linear to be zero 
                    linear_msg = UInt8()
                    linear_msg.data = 0
                    self.linear_publisher.publish(linear_msg)
                    
                    # angle_min > or < 180, the delta angle to move away from the object is still the same
                    angle_msg = Float64()
                    angle_msg.data = angle_min - 180.0
                    self.angle_publisher.publish(angle_msg)
                else:
                    # move away until its more than 30 cm
                    # set linear to be 1 
                    linear_msg = UInt8()
                    linear_msg.data = 1
                    self.linear_publisher.publish(linear_msg)
                    
                    # set delta angle = 0
                    angle_msg = Float64()
                    angle_msg.data = 0.0
                    self.angle_publisher.publish(angle_msg)
                    
                    
                    
        elif self.state == "moving_away_from_walls":
            pass
        elif self.state == "locating_bucket":
            self.get_logger().info('self.state %s' % self.state)

            # lidar minimum is 12 cm send by node, datasheet says 16 cm
            # by experimentation need 30 cm
            # if less than 30 cm from nearest object, move away from it, else can find the bucket using bucketFinderNode
            min_distance = min(self.laser_range)
            
            self.get_logger().info('min_distance %f' % min_distance)
            
            if min_distance < 0.3:
                self.get_logger().info('too close! moving away')
                
                argmin = np.nanargmin(self.laser_range)
                angle_min = self.index_to_angle(argmin, self.range_len)
                
                self.get_logger().info('angle_min %f' % angle_min)
                
                # if the closest object is in not at the back of the robot, rotate first, else move away from it
                if abs(angle_min - 180) > 5:
                    # set linear to be zero 
                    linear_msg = UInt8()
                    linear_msg.data = 0
                    self.linear_publisher.publish(linear_msg)
                    
                    # angle_min > or < 180, the delta angle to move away from the object is still the same
                    angle_msg = Float64()
                    angle_msg.data = angle_min - 180.0
                    self.angle_publisher.publish(angle_msg)
                else:
                    # move away until its more than 30 cm
                    # set linear to be 1 
                    linear_msg = UInt8()
                    linear_msg.data = 1
                    self.linear_publisher.publish(linear_msg)
                    
                    # set delta angle = 0
                    angle_msg = Float64()
                    angle_msg.data = 0.0
                    self.angle_publisher.publish(angle_msg)
            else:
                # robot is far enough away from everything to not cause false positive
                self.get_logger().info('far away enough! go to rotating_to_bucket state')
                
                # set movement to zero
                # set linear to be zero 
                linear_msg = UInt8()
                linear_msg.data = 0
                self.linear_publisher.publish(linear_msg)
                
                # set delta angle = 0
                angle_msg = Float64()
                angle_msg.data = 0.0
                self.angle_publisher.publish(angle_msg)
                
                # go to next state
                self.state = "rotating_to_bucket"
        elif self.state == "rotating_to_bucket":
            self.get_logger().info('self.state %s' % self.state)
            
            # if close to forward, go to next state, else allign to bucket first
            if abs(self.bucketAngle) < 2:
                self.get_logger().info('close enough, moving to bucket now')
                self.state = "moving_to_bucket"
                
                # set movement to zero
                # set linear to be zero 
                linear_msg = UInt8()
                linear_msg.data = 0
                self.linear_publisher.publish(linear_msg)
                
                # set delta angle = 0
                angle_msg = Float64()
                angle_msg.data = 0.0
                self.angle_publisher.publish(angle_msg)
            else:
                if self.bucketAngle < 180:
                    # set linear to be zero 
                    linear_msg = UInt8()
                    linear_msg.data = 0
                    self.linear_publisher.publish(linear_msg)
                    
                    # set delta angle = bucketAngle
                    angle_msg = Float64()
                    angle_msg.data = self.bucketAngle * 1.0
                    self.angle_publisher.publish(angle_msg)
                elif self.bucketAngle > 180:
                    # set linear to be zero 
                    linear_msg = UInt8()
                    linear_msg.data = 0
                    self.linear_publisher.publish(linear_msg)
                    
                    # set delta angle = bucketAngle -360
                    angle_msg = Float64()
                    angle_msg.data = self.bucketAngle - 360.0
                    self.angle_publisher.publish(angle_msg)
                else:
                    # the case where it is 180, turn 90 deg first
                    # set linear to be zero 
                    linear_msg = UInt8()
                    linear_msg.data = 0
                    self.linear_publisher.publish(linear_msg)
                    
                    # set delta angle = 90
                    angle_msg = Float64()
                    angle_msg.data = 90.0
                    self.angle_publisher.publish(angle_msg)
        elif self.state == "moving_to_bucket":
            pass
        elif self.state == "releasing":
            pass
        else:
            self.state = "idle"


def main(args=None):
    rclpy.init(args=args)

    master_node = MasterNode()

    try:
        rclpy.spin(master_node)
    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()


if __name__ == '__main__':
    main()
