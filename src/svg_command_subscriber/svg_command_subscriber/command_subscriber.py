import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Int32
from collections import deque

class SVGExecutorNode(Node):
    def __init__(self):
        super().__init__('svg_command_subscriber')

        # Subscriptions
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

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/move_group/goal', 10)

        # Internal state
        self.pose_queue = deque()
        self.executing = False

        self.get_logger().info('SVGExecutorNode initialized and ready.')

    def pose_array_callback(self, msg: PoseArray):
        self.get_logger().info(f'[pose_array_callback] Received {len(msg.poses)} poses.')

        if len(msg.poses) == 0:
            self.get_logger().warn('[pose_array_callback] Received an empty pose array.')
            return

        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = msg.header.frame_id
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose
            self.pose_queue.append(pose_stamped)

        self.get_logger().info(f'[pose_array_callback] Total poses in queue: {len(self.pose_queue)}')

    def status_callback(self, msg: Int32):
        self.get_logger().info(f'[status_callback] Received ur_status: {msg.data}')

        if msg.data == 0 and not self.executing and self.pose_queue:
            next_pose = self.pose_queue.popleft()

            # Publish the next pose
            self.pose_pub.publish(next_pose)
            self.get_logger().info(
                f'[status_callback] Published pose to /move_group/goal → '
                f'x={next_pose.pose.position.x:.3f}, '
                f'y={next_pose.pose.position.y:.3f}, '
                f'z={next_pose.pose.position.z:.3f}'
            )

            self.executing = True

        elif msg.data == 1:
            self.executing = True
            self.get_logger().info('[status_callback] Robot is executing a motion.')

        elif msg.data == 2:
            self.executing = False
            self.get_logger().info('[status_callback] Robot has completed a motion.')

        elif msg.data == 3:
            self.executing = False
            self.get_logger().warn('[status_callback] Robot encountered a singularity.')

        elif msg.data == 4:
            self.executing = False
            self.get_logger().error('[status_callback] Robot motion failed.')

def main(args=None):
    rclpy.init(args=args)
    node = SVGExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down SVGExecutorNode...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
