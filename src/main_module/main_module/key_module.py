import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import Twist, TransformStamped

from .utils.mqtt_util import MQTT
from .utils.topics import KEY_MQTT, CMD_MQTT_KEY, POSE_TOPIC
import struct

from tf_transformations import euler_from_quaternion

class key_module(Node):
    def __init__(self, mqtt):
        super().__init__('key_node')
        print(KEY_MQTT)
        self.mqtt = mqtt
        self.mqtt.mqttclient.message_callback_add(KEY_MQTT, self.on_key_received)
        self.keypub = self.create_publisher(Twist, 'cmd_vel', 10)

    def on_key_received(self, client, userdata, msg):
        self.get_logger().info('Received command "%s"' % msg.payload)
        temp = struct.unpack("{}d".format(2), msg.payload[0:16])
        print(temp)

        twist = Twist()
        twist.linear.x = float(temp[0])
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(temp[1])

        self.keypub.publish(twist)

           



def main(args=None):
    rclpy.init(args=args)

    mqtt = MQTT()
    ros_node = key_module(mqtt)

    rclpy.spin(ros_node)

    ros_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
