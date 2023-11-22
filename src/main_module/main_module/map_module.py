import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from nav2_msgs.srv import SaveMap

#sudo apt install ros-humble-tf-transformations
#sudo pip3 install transforms3d
from tf_transformations import euler_from_quaternion
from .utils.mqtt_util import MQTT
from .utils.topics import CMD_MQTT_MAP, POSE_MQTT, RESULT_MQTT
from .utils.topics import POSE_TOPIC, MAP_TOPIC, MAP_CONTENT_MQTT
import struct

class map_module(Node):
    def __init__(self, mqtt):
        super().__init__('map_node')
        self.saveMapBool = False
        self.pose = (0.0, 0.0)
        self.origin = (0.0, 0.0)
        self.rotation = 0.0

        self.mqtt = mqtt
        self.mqtt.mqttclient.message_callback_add(CMD_MQTT_MAP, self.on_cmd_received)

        self.subscription_pose = self.create_subscription(
            TransformStamped,
            POSE_TOPIC,
            self.publish_pose,
            qos_profile_system_default)
        
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            MAP_TOPIC,
            self.publish_map,
            qos_profile_system_default)
        
        self.timer = self.create_timer(1, self.timerTest)

    def timerTest(self):
        if (self.saveMapBool == True):
            self.saveMapBool = False
            self.saveCurrentMap()
        self.get_logger().info('Pose publisher')
        ioPose = [sum(x) for x in zip(self.pose,self.origin)]
        ioPoint = list(ioPose) + list(self.origin)
        ioPoint.append(self.rotation)
        print(ioPoint)
        mqttPose = struct.pack(">{}f".format(len(ioPoint)), *ioPoint)
        self.mqtt.mqttclient.publish(POSE_MQTT, mqttPose)
            
        
    def publish_pose(self, msg):
        self.get_logger().info('Received pose')
        self.pose = list(x * -1 for x in (msg.transform.translation.x, msg.transform.translation.y))
        rotation = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
        (roll, pitch, yaw) = euler_from_quaternion(rotation)
        self.rotation = yaw
        #self.ioPose = struct.pack(">{}f".format(len(pose)), *pose)
        #self.mqtt.mqttclient.publish(POSE_MQTT, self.ioPose)

    def publish_map(self, msg):
        self.get_logger().info('Received map')
        self.origin = list((msg.info.origin.position.x, msg.info.origin.position.y))
        #self.ioMapOrigin = struct.pack(">{}f".format(len(origin)), *origin)
        #self.mqtt.mqttclient.publish(MAP_ORIGIN_MQTT, self.ioMapOrigin)

        info = list((msg.info.width, msg.info.height))
        temp = list(msg.data)
        data = info + temp
        self.ioMapData = struct.pack(">{}h".format(len(data)), *data)
        self.mqtt.mqttclient.publish(RESULT_MQTT, "Ready!")
        
        #self.mqtt.mqttclient.publish(MAP_CONTENT_MQTT, self.ioMapData)
        
    def on_cmd_received(self, client, userdata, msg):
        self.get_logger().info('Received command "%s"' % msg.payload)
        #print("Received command {}".format(msg.payload))
        if msg.payload.decode("utf-8") == "savemap":
            self.saveMapBool = True
        elif msg.payload.decode("utf-8") == "update":
            #self.mqtt.mqttclient.publish(MAP_ORIGIN_MQTT, self.ioMapOrigin)
            self.mqtt.mqttclient.publish(MAP_CONTENT_MQTT, self.ioMapData)      

    def saveCurrentMap(self):
        print("Saving map...")
        self.cli = self.create_client(SaveMap, '/map_saver/save_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SaveMap.Request()
        req = SaveMap.Request()
        req.map_url = 'test_map'
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        req.map_topic = 'map'
        req.map_mode = 'trinary'
        req.image_format = 'pgm'

        # double calls since it keep saving default map if only one call is used
        self.cli.call_async(req)
        self.cli.call_async(req)

        #rclpy.spin_until_future_complete(self, self.future)
        #rclpy.spin_once(self.cli.call_async(self.req))
        #return self.future.result()
        

def main(args=None):
    rclpy.init(args=args)
    mqtt = MQTT()
    ros_node = map_module(mqtt)

    rclpy.spin(ros_node)

    ros_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
