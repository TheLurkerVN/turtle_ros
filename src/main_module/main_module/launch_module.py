import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

import threading
import subprocess

from .utils.mqtt_util import MQTT
from .utils.topics import CMD_MQTT_MODE
import os

class threadObject(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.process = None

    def setMode(self, mode):
        if mode == "nav":
            self.cmd = "ros2 launch turtlebot3_app_launcher navigation_app.launch.py"
        elif mode == "slam":
            self.cmd = "ros2 launch turtlebot3_app_launcher cartographer_app.launch.py"

    def run(self):
        self.process = p = subprocess.Popen([self.cmd], shell=True)

    def stop(self):
        if self.process is not None:
            self.process.terminate()
            self.process = None

        
class launch_module(Node):
    def __init__(self):
        super().__init__('launch_node')
        self.mqtt = MQTT()
        self.mqtt.mqttclient.message_callback_add(CMD_MQTT_MODE, self.on_cmd_received)
        self.proc = None
        #self.thread = threadObject()
        #threading.Thread
        #self.proc = subprocess.Popen(['ros2 launch turtlebot3_app_launcher cartographer_app.launch.py'], shell=True)

    def on_cmd_received(self, client, userdata, msg):
        print("Received command {}".format(msg.payload))
        if self.proc is not None:
            self.proc.terminate()
            os.system("sleep 10")
            
        if msg.payload.decode("utf-8") == "navigation_mode":
            print("Test")
            self.proc = subprocess.Popen("exec " + "ros2 launch turtlebot3_app_launcher navigation_app.launch.py", shell=True)
            #self.thread.stop()
            #self.thread.setMode("nav")
            #self.thread.start()
            #self.proc = subprocess.Popen(['ros2 launch turtlebot3_app_launcher navigation_app.launch.py'], shell=True)
            #os.system("ros2 launch turtlebot3_app_launcher navigation_app.launch.py")

        elif msg.payload.decode("utf-8") == "slam_mode":
            #self.thread.stop()
            #self.thread.setMode("slam")
            #self.thread.start()
            #self.proc.terminate()
            #os.system("ros2 launch turtlebot3_app_launcher cartographer_app.launch.py")
            self.proc = subprocess.Popen("exec " + "ros2 launch turtlebot3_app_launcher cartographer_app.launch.py", shell=True)

def main(args=None):
    rclpy.init(args=args)

    ros_node = launch_module()

    rclpy.spin(ros_node)

    ros_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()