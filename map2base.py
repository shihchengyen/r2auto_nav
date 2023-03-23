import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose
import numpy as np
from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener


#This program will publish a topic /map2base which will be the information of the transform from tf /map to /base_link, this will allow
#us to determine the robots location and rotation in the map.

class Map2Base(Node):

    def __init__(self):
        super().__init__('map2base')
        self.declare_parameter('target_frame', 'base_footprint')
        self.target_frame = self.get_parameter(
        'target_frame').get_parameter_value().string_value
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self, spin_thread=True)
        self.mapbase = None
        self.map2base = self.create_publisher(Pose, '/map2base', 10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # create numpy array
        msg = Pose()
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'
        now = rclpy.time.Time()
        try:
            # while not self.tf_buffer.can_transform(to_frame_rel, from_frame_rel, now, timeout = Duration(seconds=1.0)):
            self.mapbase = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
                        # ,
                        # timeout = Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        msg.position.x = self.mapbase.transform.translation.x 
        msg.position.y = self.mapbase.transform.translation.y
        msg.orientation = self.mapbase.transform.rotation
        
        self.map2base.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    map2base = Map2Base()

    rclpy.spin(map2base)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map2base.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
