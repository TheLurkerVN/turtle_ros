import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import BatteryState

from .utils.mqtt_util import MQTT
from .utils.topics import KEY_MQTT, BATTERY_TOPIC, BATTERY_MQTT
import struct

from tf_transformations import euler_from_quaternion

class key_module(Node):
    def __init__(self, mqtt):
        super().__init__('key_node')

        self.bat = 100

        self.mqtt = mqtt
        self.mqtt.mqttclient.message_callback_add(KEY_MQTT, self.on_key_received)
        self.keypub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_battery = self.create_subscription(
            BatteryState,
            BATTERY_TOPIC,
            self.get_battery,
            qos_profile_system_default)
        
        self.batterytimer = self.create_timer(5, self.publish_battery)

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

    def get_battery(self, msg):
        self.bat = msg.percentage

    def publish_battery(self):
        self.mqtt.mqttclient.publish(BATTERY_MQTT, self.bat)


def main(args=None):
    rclpy.init(args=args)

    mqtt = MQTT()
    ros_node = key_module(mqtt)

    rclpy.spin(ros_node)

    ros_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
