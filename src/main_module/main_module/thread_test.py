import rclpy
from rclpy.executors import MultiThreadedExecutor

from .utils.mqtt_util import MQTT

from .key_module import key_module
from .map_module import map_module

def main(args=None):
    rclpy.init(args=args)
    mqtt = MQTT()

    key = key_module(mqtt)
    map = map_module(mqtt)

    exec = MultiThreadedExecutor()

    exec.add_node(key)
    exec.add_node(map)

    exec.spin()
    exec.shutdown()

    key.destroy_node()
    map.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

