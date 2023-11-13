import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

import subprocess

from .utils.mqtt_util import MQTT
from .utils.topics import CMD_MQTT_MODE
import os
        
class launch_module(Node):
    def __init__(self):
        super().__init__('launch_node')
        self.mqtt = MQTT()
        self.mqtt.mqttclient.message_callback_add(CMD_MQTT_MODE, self.on_cmd_received)
        self.proc = None

    def on_cmd_received(self, client, userdata, msg):
        print("Received command {}".format(msg.payload))
        if self.proc is not None:
            self.proc.terminate()
            os.system("sleep 10")

        if msg.payload.decode("utf-8") == "navigation_mode":
            self.proc = subprocess.Popen("exec " + "ros2 launch turtlebot3_app_launcher navigation_app.launch.py", shell=True)
        elif msg.payload.decode("utf-8") == "slam_mode":
            self.proc = subprocess.Popen("exec " + "ros2 launch turtlebot3_app_launcher cartographer_app.launch.py", shell=True)

def main(args=None):
    rclpy.init(args=args)

    ros_node = launch_module()

    rclpy.spin(ros_node)

    ros_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()