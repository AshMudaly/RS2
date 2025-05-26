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

<<<<<<< HEAD
        # Load and process SVG
        self.svg_path = os.path.expanduser('~/ros2_ws/RS2/output.svg')

        if not os.path.exists(self.svg_path):
            self.get_logger().error(f'SVG file not found: {self.svg_path}')
            return
        
        self.publish_markers_and_poses()
=======
        svg_path = '/home/ashmu/ros2_ws/RS2/output.svg'
>>>>>>> 6de09c8 (ok re run)

        if not os.path.exists(svg_path):
            self.get_logger().error(f"SVG file not found: {svg_path}")
            return

        self.process_and_publish(svg_path)

    def process_and_publish(self, svg_path):
        paths, _ = svg2paths(svg_path)
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'

        for path in paths:
            points = []
            for segment in path:
                for t in np.linspace(0, 1, num=20):
                    pt = segment.point(t)
                    points.append([pt.real / 100.0, pt.imag / 100.0])  # scale to meters

            simplified = rdp(points, epsilon=0.005)

            for pt in simplified:
                pose = Pose()
                pose.position.x = pt[0]
                pose.position.y = pt[1]
                pose.position.z = 0.1
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)

        self.pose_array_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} poses to /svg_waypoints/poses.")

def main(args=None):
    rclpy.init(args=args)
    node = SVGWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
