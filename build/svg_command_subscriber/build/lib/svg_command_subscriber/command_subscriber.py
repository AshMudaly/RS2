import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import tf_transformations
import time

class SVGCommandSubscriber(Node):
    def __init__(self):
        super().__init__('svg_command_subscriber')
        self.pose_publisher = self.create_publisher(PoseStamped, '/move_group/goal', 10) # ADJUST TOPIC
        self.waypoint_subscriber = self.create_subscription(
            Marker,
            '/svg_waypoints',
            self.waypoint_callback,
            10
        )

        self.declare_parameter('fixed_z', 0.1)
        self.declare_parameter('orientation_x', 0.0)
        self.declare_parameter('orientation_y', 0.0)
        self.declare_parameter('orientation_z', 0.0)
        self.declare_parameter('orientation_w', 1.0)

        self.fixed_z = self.get_parameter('fixed_z').get_parameter_value().double_value
        self.orientation = [
            self.get_parameter('orientation_x').get_parameter_value().double_value,
            self.get_parameter('orientation_y').get_parameter_value().double_value,
            self.get_parameter('orientation_z').get_parameter_value().double_value,
            self.get_parameter('orientation_w').get_parameter_value().double_value
        ]

        self.get_logger().info("SVG Command Subscriber Node Started, listening for waypoints...")

    def waypoint_callback(self, msg):
        if msg.type == Marker.SPHERE:  # Assuming your waypoints are published as SPHERE markers
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'  # Or the frame_id of your waypoints

            # Extract position from the Marker message
            pose_msg.pose.position.x = msg.pose.position.x
            pose_msg.pose.position.y = msg.pose.position.y
            pose_msg.pose.position.z = self.fixed_z  # Use the fixed Z parameter

            # Use the configured orientation
            pose_msg.pose.orientation.x = self.orientation[0]
            pose_msg.pose.orientation.y = self.orientation[1]
            pose_msg.pose.orientation.z = self.orientation[2]
            pose_msg.pose.orientation.w = self.orientation[3]

            self.get_logger().info(f"Received waypoint, publishing pose: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
            self.pose_publisher.publish(pose_msg)
            time.sleep(0.1) # Small delay between publishing poses

def main(args=None):
    rclpy.init(args=args)
    node = SVGCommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
