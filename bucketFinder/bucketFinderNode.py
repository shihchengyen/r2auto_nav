import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

class BucketFinderHandler(Node):
    def __init__(self):
        super().__init__('bucketFinderNode')
        
        # Create a subscriber to the topic "scan"
        # Listens for the lidar distances
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning  
        
        # Create a publisher to the topic "bucketAngle"
        # Publishes the angle of the bucket to the main control node
        self.publisher_ = self.create_publisher(UInt16, 'bucketAngle', 10)
        publisher_period = 0.5  # seconds
        self.publisherTimer = self.create_timer(publisher_period, self.publisher_callback)
        
        # to store lidar data and valid min/max values
        self.laser_range = []
        self.range_min = 0.0
        self.range_max = 0.0
        self.range_len = 0
        
        self.get_logger().info("bucketFinderNode has started! :D")

    def listener_callback(self, msg):
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
    
    def publisher_callback(self):
        
        
        # msg = UInt16()
        
        # # Bucket detection code here 
        # # For now, just publishing 0
        # bucketAngle = 365
        # msg.data = bucketAngle
        # # msg.data = np.argmin(self.laser_range)
        
        # self.publisher_.publish(msg)
        # self.get_logger().info("Publishing bucketAngle: %s" % msg.data)
        
        # self.get_logger().info("len(self.laser_range): %s" % str(len(self.laser_range)))
        # self.get_logger().info(str(self.index_to_angle(len(self.laser_range), len(self.laser_range))))
        
        self.drdtheta = self.dr_dtheta(self.laser_range)
        # self.get_logger().info("len(drdtheta): %s" % str(len(drdtheta)))
        
        # Get the indices that would sort the array
        sorted_indices = np.argsort(self.drdtheta)

        # The last two indices in this array are the indices of the two maximum values
        max1_index, max2_index = sorted_indices[-2:]

        # bucket is in the middle of the two max values
        bucket_index = (max1_index + max2_index) // 2
        
        bucket_angle = self.index_to_angle(bucket_index, len(self.drdtheta))
        
        self.get_logger().info('Bucket: %i deg' % bucket_angle)

        
        # # find index with minimum value
        # lr2i = np.nanargmin(self.laser_range)
        # lr2a = self.index_to_angle(lr2i, self.range_len)
        # self.get_logger().info('Shortest distance at %i degrees' % lr2a)
        
    def index_to_angle(self, index, arrLen):
        return (index / (arrLen - 1)) * 359
    
    def dr_dtheta(self, lidarArr):
        drdindex = np.array([((lidarArr[(i + 1) % len(lidarArr)] - lidarArr[i - 1]) / 2.0) for i in range(len(lidarArr))])
        drdtheta = drdindex * (len(lidarArr) - 1) / 359.0
        return drdtheta
    
    def plot_lidar_readings(self):
        self.get_logger().info(str(self.laser_range))
        
        # Create an array of angles to match the ranges
        angles = np.linspace(0, 2*np.pi, len(self.laser_range))

        # Create a polar plot
        plt.figure()
        ax = plt.subplot(111, projection='polar')
    
        # Plot the laser ranges
        ax.plot(angles, self.laser_range, 'o', markersize=1, label='Laser Ranges')

        # Plot drdtheta
        ax.plot(angles, self.drdtheta, 'x', markersize=1, label='drdtheta')

        # Add a legend
        ax.legend()

        # Display the plot
        plt.show()
    
def main(args=None):
    rclpy.init(args=args)

    bucketFinder = BucketFinderHandler()

    try:
        rclpy.spin(bucketFinder)
    except KeyboardInterrupt:
        pass
    finally:
        bucketFinder.plot_lidar_readings()
        bucketFinder.destroy_node()

if __name__ == '__main__':
    main()
