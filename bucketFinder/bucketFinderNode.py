import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

import argparse

class BucketFinderHandler(Node):
    def __init__(self, show_plot):
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
    
        self.show_plot = show_plot
        

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
        
        if all(np.isnan(self.laser_range)):
            self.get_logger().info("No valid lidar data")
            self.bucket_angle = np.nan
            
        else:
            self.drdtheta = self.dr_dtheta(self.laser_range)
            # self.get_logger().info("len(drdtheta): %s" % str(len(drdtheta)))
            
            # The last two indices in this array are the indices of the two maximum values
            self.max_index = np.nanargmax(self.drdtheta)
            self.min_index = np.nanargmin(self.drdtheta)
            
            self.min_angle = self.index_to_angle(self.min_index, len(self.drdtheta))
            self.max_angle = self.index_to_angle(self.max_index, len(self.drdtheta))

            # bucket is in the middle of the two max values, with care taken if it crosses theta = 0, 
            # returns nan if min and max defer by more than 90 degrees, or less than 10 degress
            if self.max_angle < 90 and self.min_angle > 270:
                if abs(self.min_angle - self.max_angle - 360) > 180 or abs(self.min_angle - self.max_angle - 360) < 10:
                    self.bucket_angle = np.nan
                else:
                    temp = (self.min_angle + self.max_angle - 360) // 2
                    
                    if temp < 0:
                        self.bucket_angle = temp + 360
                    else:
                        self.bucket_angle = temp
            else:
                if abs(self.max_angle - self.min_angle) > 180 or abs(self.max_angle - self.min_angle) < 10:
                    self.bucket_angle = np.nan
                else:
                    self.bucket_angle = (self.min_angle + self.max_angle) // 2
                
            
            # self.bucket_angle = self.index_to_angle(bucket_index, len(self.drdtheta))
        
            self.get_logger().info('min_index  : %i' % self.min_index)
            self.get_logger().info('max_index  : %i' % self.max_index)
            self.get_logger().info('min_angle  : %i deg' % self.min_angle)
            self.get_logger().info('max_angle  : %i deg' % self.max_angle)
            if np.isnan(self.bucket_angle):
                self.get_logger().info('bucket_angle: nan')
            else:
                self.get_logger().info('bucket_angle: %i deg' % self.bucket_angle)

            # # find index with minimum value
            # lr2i = np.nanargmin(self.laser_range)
            # lr2a = self.index_to_angle(lr2i, self.range_len)
            # self.get_logger().info('Shortest distance at %i degrees' % lr2a)
            
            if self.show_plot == 'y':
                self.plot_lidar_readings()
                
        # only publish if its not NaN
        if not np.isnan(self.bucket_angle):
            msg = UInt16()
            msg.data = int(self.bucket_angle)
            self.publisher_.publish(msg)
        
    def index_to_angle(self, index, arrLen):
        # return in degrees
        return (index / (arrLen - 1)) * 359
    
    def dr_dtheta(self, lidarArr):
        drdindex = np.array([((lidarArr[(i + 1) % len(lidarArr)] - lidarArr[i - 1]) / 2.0) for i in range(len(lidarArr))])
        drdtheta = drdindex * (len(lidarArr) - 1) / 359.0
        return drdtheta
    
    def plot_lidar_readings(self):
        # self.get_logger().info(str(self.laser_range))
        
        # # Create an array of angles to match the ranges
        # angles = np.linspace(0, 2*np.pi, len(self.laser_range))

        # # Create a polar plot
        # plt.figure()
        # ax = plt.subplot(111, projection='polar')
    
        # # Plot the laser ranges
        # ax.plot(angles, self.laser_range, 'o', markersize=1, label='Laser Ranges')

        # # Plot drdtheta
        # ax.plot(angles, self.drdtheta, 'x', markersize=1, label='drdtheta')
        
        # ax.set_ylim(-1, 1)  # Replace 0 and 10 with the desired minimum and maximum radial limits


        # # Add a legend
        # ax.legend()

        # Display the plot
        # plt.show()
        
        # plt.figure()
        # plt.polar(np.arange(0, 360, 360/len(self.laser_range))/180*np.pi, self.laser_range)
        # plt.title('Laser Ranges')


        # plt.figure()
        # plt.polar(np.arange(0, 360, 360/len(self.drdtheta))/180*np.pi, self.drdtheta)
        # plt.title('drdtheta')
        
        # Clear the figure
        plt.clf()   
        
        # plotting r(theta)
        plt.polar(np.arange(0, 360, 360/len(self.laser_range))/180*np.pi, self.laser_range, label='laser_range')
        
        # plotting dr/dtheta
        # take absolute to not plot negative value and times 2 to scale is so its visible in one plot with self.laser_range
        plt.polar(np.arange(0, 360, 360/len(self.drdtheta))/180*np.pi, 2 * np.abs(self.drdtheta), label='drdtheta')
        
        # plotting direciton of the bucket
        line = np.linspace(0, max(self.laser_range), 10)
        
        # no need plot line if its nan
        if not np.isnan(self.bucket_angle):
            lineAngle = np.empty(10)
            lineAngle.fill(self.bucket_angle*np.pi/180)
            plt.polar(lineAngle, line, label='bucket_angle')
        
        plt.legend()
        
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

        # plt.show()
        
        
    
def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Start the bucketFinderNode.')
    parser.add_argument('-s', type=str, default='n', help='Show plot (y/n)')
    args = parser.parse_args()
    
    bucketFinder = BucketFinderHandler(args.s)
    
    if args.s == 'y':
        # create matplotlib figure
        plt.ion()
        plt.figure()    
    
    try:
        rclpy.spin(bucketFinder)
    except KeyboardInterrupt:
        pass
    finally:
        # bucketFinder.plot_lidar_readings()
        bucketFinder.destroy_node()

if __name__ == '__main__':
    main()
