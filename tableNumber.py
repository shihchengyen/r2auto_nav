from std_msgs.msg import String
import time
import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile
import paho.mqtt.client as mqtt

class SubscribeMqtt(Node):

    def __init__(self):
        super().__init__('subscribe_mqtt')
        State = QoSProfile(durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=qos.QoSReliabilityPolicy.RELIABLE, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=20)
        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', None),
                ('port', None)])
        self.ros2_publish = self.create_publisher(
            String, '/tableNumber', Streaming)

        self.client = mqtt.Client()
        self.client.on_message = self.topic_get
        self.host = "172.20.10.3"
        self.port = 1883
        self.client.connect(self.host, self.port)
        self.client.subscribe("ESP32/tableNumber", qos=1) #to subscribe more than one topic add more subscribe lines
        self.client.loop_forever()

    def topic_get(self, client, userdata, msg):
        send = String()
        topic = msg.topic
        message = msg.payload.decode("utf-8")
        send.data = message
        self.get_logger().info('Publishing: "%s"' % message)

        if topic == "EPS32/tableNumber":
            self.ros2_publish.publish(send)


def main(args=None):
    rclpy.init(args=args)
    subscribe_mqtt = SubscribeMqtt()
    rclpy.spin(subscribe_mqtt)

    subscribe_mqtt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
