#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('ur3e_spawn_node')
        self.get_logger().info('UR3e is going to spawn!')

        # Create a timer to call the timer_callback function every second
        #self.timer = self.create_timer(1.0, self.timer_callback)

   # def timer_callback(self):
    #    self.get_logger().info('Timer callback called!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)

    # code goes here

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()