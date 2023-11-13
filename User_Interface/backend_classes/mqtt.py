import paho.mqtt.client as mqtt
import backend_classes.topics as topics

MQTT_SERVER = "192.168.1.12"
MQTT_USERNAME = "amc"
MQTT_PASSWORD = "lurker"

#MQTT_PATH = "key_parameters"
#MAP_ORIGIN = "map_origin"
#MAP_CONTENT = "map_content"
#POSE_MQTT = "pose_mqtt"
#STATUS_MQTT = "status_mqtt"
#BATTERY_MQTT = "battery_mqtt"

class MQTTClient():
    def __init__(self):
        self.client = mqtt.Client()
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect_async(MQTT_SERVER, 1883)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.client.subscribe(topics.MAP_ORIGIN_MQTT)
        self.client.subscribe(topics.MAP_CONTENT_MQTT)
        self.client.subscribe(topics.POSE_MQTT)
        self.client.subscribe(topics.BATTERY_MQTT)

    def on_message(self, client, userdata, rc):
        print("Message received")

    def publishControl(self, topic, param):
        self.client.publish(topic, param)