import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# imported from https://github.com/wr1159/r2auto_nav/blob/main/map2base.py

# This program will publish a topic /position which will be the information of the transform from tf /map to /base_link.
# This will allow us to determine the robots location and rotation in the map.

class PositionTrackNode(Node):

    def __init__(self):
        super().__init__('positionTrackNode')
        self.declare_parameter('target_frame', 'base_footprint')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.mapbase = None
        self.map2base = self.create_publisher(Pose, '/position', 10)
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
        # self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    position_traker = PositionTrackNode()

    try:
        rclpy.spin(position_traker)
    except KeyboardInterrupt:
        pass
    finally:
        position_traker.destroy_node()


if __name__ == '__main__':
    main()
