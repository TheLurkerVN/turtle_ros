import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Path

from .utils.mqtt_util import MQTT
from .utils.topics import NAV_MQTT, CMD_MQTT_NAV, BATTERY_MQTT, BATTERY_TOPIC, POSE_TOPIC
import struct

#rclpy.init()

class navigation_module(Node):
    def __init__(self, mqtt):
        super().__init__('navigation_node')
        self.controlString = list("000")
        self.navPos = (0.0, 0.0)
        self.mqtt = mqtt
        self.mqtt.mqttclient.message_callback_add(NAV_MQTT, self.on_nav_received)
        self.mqtt.mqttclient.message_callback_add(CMD_MQTT_NAV, self.on_cmd_received)
        self.navigator = Nav2Navigator()
        self.navigator.changeToNavMode()
        self.timer = self.create_timer(1, self.timerTest)
        self.subscription_battery = self.create_subscription(
            BatteryState,
            BATTERY_TOPIC,
            self.check_battery,
            qos_profile_system_default)

    def timerTest(self):
        
        if (self.controlString[0] == '1'):
            self.controlString[0] = '0'
            self.navigator.goToPose(self.navPos)
        if (self.controlString[1] == '1'):
            self.controlString[1] = '0'
            self.navigator.cancelNav()

    def check_battery(self, msg):
        self.battery = msg.percentage
        if (self.battery < 10):
            self.controlString[1] == '1'
        
        
    def on_nav_received(self, client, userdata, msg):
        if (self.battery > 10):
            self.controlString[0] = '1'
            self.navPos = struct.unpack("{}d".format(2), msg.payload[0:16])

    def on_cmd_received(self, client, userdata, msg):
        self.get_logger().info('Received command "%s"' % msg.payload)
        #print("Received command {}".format(msg.payload))
        if msg.payload.decode("utf-8") == "cancelnav":
            self.controlString[1] = '1'
            
        

class Nav2Navigator():
    def __init__(self):
        
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

    def goToPose(self, pos):
        print("startNavigation")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = pos[0]
        goal_pose.pose.position.y = pos[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        self.nav.goToPose(goal_pose)

    def cancelNav(self):
        print("cancel")
        self.nav.cancelTask()

    def changeToNavMode(self):
        print("initPose")
        self.nav.changeMap('test_map.yaml')   #map.yaml for Sim

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0     #-2.0, -0.5 for Sim
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0

        self.nav.setInitialPose(initial_pose)

        

def main(args=None):
    rclpy.init()
    mqtt = MQTT()
    ros_node = navigation_module(mqtt)
    
    rclpy.spin(ros_node)

    ros_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()