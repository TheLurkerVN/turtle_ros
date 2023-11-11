import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


from .utils.mqtt_util import MQTT
from .utils.topics import CMD_MQTT_SLAM, CMD_MQTT_NAV_MODE
import os

class slam_launch_module(Node):
    def __init__(self):
        super().__init__('nav_launch_node')
        self.mqtt = MQTT()
        self.mqtt.mqttclient.message_callback_add(CMD_MQTT_NAV_MODE, self.on_cmd_received)

    def on_cmd_received(self, client, userdata, msg):
        print("Received command {}".format(msg.payload))
        if msg.payload.decode("utf-8") == "navigation_mode":
            os.system("ros2 launch turtlebot3_app_launcher navigation_app.launch.py")

def main(args=None):
    rclpy.init(args=args)

    ros_node = slam_launch_module()

    rclpy.spin(ros_node)

    ros_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()