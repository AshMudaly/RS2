import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Int32


class UR3ePosePublisher(Node):
    def __init__(self):
        super().__init__('ur3e_pose_publisher')
        
        self.pose_publisher = self.create_publisher(PoseStamped, '/move_group/goal', 10)
        self.status_subscriber = self.create_subscription(Int32, '/ur_status', self.status_callback, 10)
        
        self.pose_matrix = [
            self.create_pose(-0.1, 0.300, 0.139),
            self.create_pose( 0.0, 0.300, 0.139),
            self.create_pose( 0.1, 0.300, 0.139),
            self.create_pose( 0.1, 0.400, 0.139),
            self.create_pose( 0.0, 0.400, 0.139),
            self.create_pose(-0.1, 0.400, 0.139),
        ]
        
        self.current_index = 0

        self.get_logger().info("UR3e Debug Pose Publisher Node Started.")
    
    def create_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        return pose

    def status_callback(self, msg):
        status = msg.data

        if status == 0:  # Ready for next pose
            self.get_logger().info("Status 0: Ready for next waypoint.")
            self.publish_next_pose()

        elif status == 1:  # Last pose reached
            self.get_logger().info("Status 1: Pose reached successfully.")

        elif status == 2:  # In motion
            self.get_logger().info("Status 2: Robot currently moving...")

        elif status == 3:  # Singularity
            self.get_logger().warn("Status 3: Singularity encountered!")

        elif status == 4:  # Motion failed
            self.get_logger().error("Status 4: Motion execution failed!")

        else:
            self.get_logger().warn(f"Unknown status code: {status}")
    
    def publish_next_pose(self):
        if self.current_index >= len(self.pose_matrix):
            self.get_logger().info("All waypoints have been published.")
            return

        pose_msg = self.pose_matrix[self.current_index]
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_publisher.publish(pose_msg)

        self.get_logger().info(
            f"Published pose {self.current_index + 1}/{len(self.pose_matrix)} "
            f"at position: {pose_msg.pose.position}"
        )

        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = UR3ePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
