import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Int32
import time
from collections import deque

class SVGExecutorNode(Node):
    def __init__(self):
        super().__init__('svg_command_subscriber')

        self.pose_sub = self.create_subscription(
            PoseArray,
            '/svg_waypoints/poses',
            self.pose_array_callback,
            10
        )

        self.status_sub = self.create_subscription(
            Int32,
            '/ur_status',
            self.status_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/move_group/goal', 10)

        self.pose_queue = deque()
        self.robot_ready = False
        self.executing = False

    def pose_array_callback(self, msg: PoseArray):
        self.get_logger().info(f'Received {len(msg.poses)} poses.')

        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = msg.header.frame_id
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose
            self.pose_queue.append(pose_stamped)

        self.get_logger().info(f'Queued {len(self.pose_queue)} waypoints.')

    def status_callback(self, msg: Int32):
        if msg.data == 0 and not self.executing and self.pose_queue:
            # Robot is idle and we can send the next pose
            next_pose = self.pose_queue.popleft()
            self.pose_pub.publish(next_pose)
            self.get_logger().info(f'Sent pose: x={next_pose.pose.position.x:.3f}, y={next_pose.pose.position.y:.3f}')
            self.executing = True

        elif msg.data == 1:
            self.executing = True  # Robot is busy

        elif msg.data == 2:
            self.executing = False  # Completed a waypoint

def main(args=None):
    rclpy.init(args=args)
    node = SVGExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down executor node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
