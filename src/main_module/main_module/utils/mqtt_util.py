import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

import paho.mqtt.client as mqtt

from .topics import KEY_MQTT, NAV_MQTT, CMD_MQTT_KEY, CMD_MQTT_MAP, CMD_MQTT_NAV, CMD_MQTT_SLAM, CMD_MQTT_NAV_MODE

MQTT_SERVER = "10.90.229.66"
MQTT_USERNAME = "amc"
MQTT_PASSWORD = "lurker"

class MQTT():
    def __init__(self):
        print("Creating MQTT...")
        self.mqttclient = mqtt.Client()
        self.mqttclient.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD) 
        self.mqttclient.connect_async(MQTT_SERVER) 
        self.mqttclient.on_connect = self.on_connect
        self.mqttclient.on_message = self.on_message
        self.mqttclient.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.mqttclient.subscribe(KEY_MQTT)
        self.mqttclient.subscribe(NAV_MQTT)
        self.mqttclient.subscribe(CMD_MQTT_KEY)
        self.mqttclient.subscribe(CMD_MQTT_MAP)
        self.mqttclient.subscribe(CMD_MQTT_NAV)
        self.mqttclient.subscribe(CMD_MQTT_SLAM)
        self.mqttclient.subscribe(CMD_MQTT_NAV_MODE)

    def on_message(self, client, userdata, msg):
        print("Received Message")
