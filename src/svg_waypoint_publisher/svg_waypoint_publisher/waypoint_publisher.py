import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from svgpathtools import svg2paths
from rdp import rdp
import numpy as np
import os

class SVGWaypointPublisher(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisher')

        self.pose_array_pub = self.create_publisher(PoseArray, '/svg_waypoints/poses', 10)

        self.svg_path = '/home/alec/ros2_ws/src/RS2/output.svg'

        if not os.path.exists(self.svg_path):
            self.get_logger().error(f"SVG file not found: {self.svg_path}")
            rclpy.shutdown()
            return

        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = 'map'

        self.process_svg()

        # Publish repeatedly every 1 second
        self.timer = self.create_timer(1.0, self.publish_poses)

    def process_svg(self):
        paths, _ = svg2paths(self.svg_path)

        all_points = []
        for path in paths:
            points = []
            for segment in path:
                for t in np.linspace(0, 1, num=20):
                    pt = segment.point(t)
                    points.append([pt.real, pt.imag])  # raw SVG coords

            points_np = np.array(points)

            all_points.append(points_np)

        if not all_points:
            self.get_logger().warn("No points extracted from SVG.")
            return

        # Concatenate all points from all paths for bounding box
        all_points_np = np.vstack(all_points)

        # Original SVG bounding box
        min_xy = all_points_np.min(axis=0)
        max_xy = all_points_np.max(axis=0)
        svg_width = max_xy[0] - min_xy[0]
        svg_height = max_xy[1] - min_xy[1]
        svg_aspect = svg_width / svg_height if svg_height > 0 else 1.0

        # Workspace bounds
        workspace_x_min = -0.125
        workspace_x_max = 0.125
        workspace_y_min = 0.275
        workspace_y_max = 0.475
        workspace_width = workspace_x_max - workspace_x_min
        workspace_height = workspace_y_max - workspace_y_min
        workspace_aspect = workspace_width / workspace_height

        # Decide scale factor to fit SVG into workspace (preserve aspect ratio)
        if svg_aspect > workspace_aspect:
            # SVG is wider than workspace, limit by width
            scale = workspace_width / svg_width if svg_width > 0 else 1.0
        else:
            # SVG is taller or equal aspect, limit by height
            scale = workspace_height / svg_height if svg_height > 0 else 1.0

        # Prepare pose array
        self.pose_array.poses = []

        for points_np in all_points:
            # Normalize points (shift min to zero)
            normalized = points_np - min_xy

            # Scale points
            scaled = normalized * scale

            # After scaling, get new bounding box size
            scaled_width = svg_width * scale
            scaled_height = svg_height * scale

            # Center offset to put SVG centered inside workspace
            x_offset = workspace_x_min + (workspace_width - scaled_width) / 2.0
            y_offset = workspace_y_min + (workspace_height - scaled_height) / 2.0

            # Shift points to workspace center
            shifted = scaled + np.array([x_offset, y_offset])

            # Simplify path with RDP (epsilon tuned for workspace scale)
            simplified = rdp(shifted.tolist(), epsilon=0.005)

            # Convert simplified points to poses
            for pt in simplified:
                pose = Pose()
                pose.position.x = pt[0]
                pose.position.y = pt[1]
                pose.position.z = 0.19  
                pose.orientation.x = 1.0
                pose.orientation.w = 0.0
                self.pose_array.poses.append(pose)

        self.get_logger().info(f"Processed and scaled {len(self.pose_array.poses)} waypoints.")

    def publish_poses(self):
        self.pose_array.header.stamp = self.get_clock().now().to_msg()
        self.pose_array_pub.publish(self.pose_array)
        self.get_logger().info(f"Published {len(self.pose_array.poses)} poses.")

def main(args=None):
    rclpy.init(args=args)
    node = SVGWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
