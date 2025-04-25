import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from svgpathtools import svg2paths
import numpy as np
import tf_transformations


class SVGPosePublisher(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisher')
        self.publisher_ = self.create_publisher(Pose, 'ur3e_target_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_next_pose)

        # Load and process SVG
        self.poses = self.load_svg_as_poses('output.svg')
        self.index = 0

    def load_svg_as_poses(self, filename):
        poses = []
        paths, _ = svg2paths(filename)
        for path in paths:
            for i in np.linspace(0, 1, 100):
                point = path.point(i)

                pose = Pose()
                pose.position.x = point.real * 0.001  # mm to m
                pose.position.y = -point.imag * 0.001  # invert y for UR3e
                pose.position.z = 0.0  # assume flat

                # Orientation: down-facing tool (adjust if needed)
                q = tf_transformations.quaternion_from_euler(0, 3.14, 0)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]

                poses.append(pose)
        return poses

    def publish_next_pose(self):
        if self.index < len(self.poses):
            pose = self.poses[self.index]
            self.publisher_.publish(pose)
            self.get_logger().info(f'Published pose {self.index + 1}/{len(self.poses)}')
            self.index += 1
        else:
            self.get_logger().info("Finished publishing all poses.")


def main(args=None):
    rclpy.init(args=args)
    node = SVGPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()