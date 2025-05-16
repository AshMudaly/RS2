import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int32
import time

class SVGCommandSubscriberNode(Node):
    def __init__(self):
        super().__init__('svg_command_subscriber')
        self.pose_publisher = self.create_publisher(PoseStamped, '/move_group/goal', 10)
        self.status_subscriber = self.create_subscription(Int32, 'ur_status', self.status_callback, 10)
        self.waypoint_subscriber = self.create_subscription(
            MarkerArray,
            '/svg_waypoints',
            self.waypoint_callback,
            10
        )

        self.declare_parameter('fixed_z', 0.1)
        self.declare_parameter('orientation_x', 0.0)
        self.declare_parameter('orientation_y', 0.0)
        self.declare_parameter('orientation_z', 0.0)
        self.declare_parameter('orientation_w', 1.0)
        self.declare_parameter('target_frame', 'map')

        self.fixed_z = self.get_parameter('fixed_z').get_parameter_value().double_value
        self.orientation = [
            self.get_parameter('orientation_x').get_parameter_value().double_value,
            self.get_parameter('orientation_y').get_parameter_value().double_value,
            self.get_parameter('orientation_z').get_parameter_value().double_value,
            self.get_parameter('orientation_w').get_parameter_value().double_value
        ]
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.waypoints_queue = []
        self.status_flag = 1  # Initialize status as ready (assuming initial state is ready)

        self.get_logger().info("SVG Command Subscriber Node Started, listening for waypoints and status...")

    def waypoint_callback(self, msg):
        for marker in msg.markers:
            if marker.type == Marker.SPHERE:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = self.target_frame

                pose_msg.pose.position.x = marker.pose.position.x
                pose_msg.pose.position.y = marker.pose.position.y
                pose_msg.pose.position.z = self.fixed_z

                pose_msg.pose.orientation.x = self.orientation[0]
                pose_msg.pose.orientation.y = self.orientation[1]
                pose_msg.pose.orientation.z = self.orientation[2]
                pose_msg.pose.orientation.w = self.orientation[3]

                self.get_logger().info(f"Received waypoint and added to queue: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
                self.waypoints_queue.append(pose_msg)
        
        # Start processing the queue if it's not empty and we are ready
        if self.waypoints_queue and self.status_flag == 1:
            self.publish_next_pose()

    def status_callback(self, msg):
        if msg.data == 1:  # Success status received
            self.get_logger().info("Received status: Success, moving to the next waypoint.")
            self.status_flag = 1
            if self.waypoints_queue:
                self.publish_next_pose()
        elif msg.data == 2:  # Currently executing
            self.get_logger().info("Received status: Executing.")
            self.status_flag = 2
        elif msg.data == 3:  # Singularity detected
            self.get_logger().warn("Received status: Singularity detected. Skipping current waypoint.")
            self.status_flag = 1
            if self.waypoints_queue:
                self.waypoints_queue.pop(0) # Remove the problematic waypoint
                self.publish_next_pose()
        elif msg.data == 4:  # Execution failed
            self.get_logger().error("Received status: Execution failed. Attempting next waypoint.")
            self.status_flag = 1
            if self.waypoints_queue:
                self.waypoints_queue.pop(0) # Remove the failed waypoint
                self.publish_next_pose()

    def publish_next_pose(self):
        if self.waypoints_queue:
            pose_msg = self.waypoints_queue.pop(0)
            self.get_logger().info(f"Publishing goal pose: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
            self.pose_publisher.publish(pose_msg)
            self.status_flag = 2  # Set status to "in motion"
        else:
            self.get_logger().info("Waypoint queue is empty.")

def main(args=None):
    rclpy.init(args=args)
    node = SVGCommandSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()