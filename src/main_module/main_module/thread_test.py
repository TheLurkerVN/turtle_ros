import rclpy
from rclpy.executors import MultiThreadedExecutor

from .utils.mqtt_util import MQTT

from .key_module import key_module
from .map_module import map_module
from .slam_launch_module import slam_launch_module
from .navigation_module import navigation_module

def main(args=None):
    rclpy.init(args=args)
    mqtt = MQTT()

    key = key_module(mqtt)
    map = map_module(mqtt)
    #slam = slam_launch_module(mqtt)
    #nav = navigation_module(mqtt)

    
    exec = MultiThreadedExecutor()

    exec.add_node(key)
    exec.add_node(map)
    #exec.add_node(slam)
    #exec.add_node(nav)

    exec.spin()
    exec.shutdown()

    key.destroy_node()
    map.destroy_node()
    #slam.destroy_node()
    #nav.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

