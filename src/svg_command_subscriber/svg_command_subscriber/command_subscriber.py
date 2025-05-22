import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from svgpathtools import svg2paths2, Line, Arc, CubicBezier, QuadraticBezier
import os
import sys
from rdp import rdp

class SVGWaypointPublisherNode(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisher')

        self.simplified_marker_array_pub = self.create_publisher(MarkerArray, 'simplified_svg_output', 10)

        self.declare_parameter('svg_file', 'output.svg')
        self.svg_file = self.get_parameter('svg_file').get_parameter_value().string_value
        self.get_logger().info(f"Using SVG file: {self.svg_file}")

        self.declare_parameter('simplification_tolerance', 0.0005)
        self.simplification_tolerance = self.get_parameter('simplification_tolerance').get_parameter_value().double_value
        self.get_logger().info(f"Using simplification tolerance: {self.simplification_tolerance}")

        self.svg_path = os.path.join('/home/ashmu/ros2_ws/RS2', self.svg_file)

        if not os.path.exists(self.svg_path):
            self.get_logger().error(f"SVG file not found at {self.svg_path}")
            sys.exit(1)

        self.process_svg_and_publish()

    def process_svg_and_publish(self):
        markers_simplified = MarkerArray()

        paths, attributes, _ = svg2paths2(self.svg_path)
        simplified_marker_base_id = 0
        N_POINTS_PER_ARC_SEGMENT = 50

        all_points = []
        for path in paths:
            for segment in path:
                if isinstance(segment, Line):
                    all_points.append([segment.start.real, -segment.start.imag])
                    all_points.append([segment.end.real, -segment.end.imag])
                elif isinstance(segment, (Arc, CubicBezier, QuadraticBezier)):
                    for i in range(N_POINTS_PER_ARC_SEGMENT + 1):
                        t = i / N_POINTS_PER_ARC_SEGMENT
                        pt = segment.point(t)
                        all_points.append([pt.real, -pt.imag])

        if not all_points:
            self.get_logger().warn("No points found in SVG to process.")
            return

        all_points_np = np.array(all_points)
        center = np.mean(all_points_np, axis=0)
        self.get_logger().info(f"SVG center offset: {center}")

        for path, attr in zip(paths, attributes):
            color = attr.get('stroke', '').lower()
            if color != '#000000' and color != 'black':
                continue

            points_for_rdp = []

            for segment in path:
                if isinstance(segment, Line):
                    x1 = segment.start.real - center[0]
                    y1 = -segment.start.imag - center[1]
                    x2 = segment.end.real - center[0]
                    y2 = -segment.end.imag - center[1]
                    points_for_rdp.extend([[x1, y1], [x2, y2]])
                elif isinstance(segment, (Arc, CubicBezier, QuadraticBezier)):
                    for i in range(N_POINTS_PER_ARC_SEGMENT + 1):
                        t = i / N_POINTS_PER_ARC_SEGMENT
                        pt = segment.point(t)
                        x = pt.real - center[0]
                        y = -pt.imag - center[1]
                        points_for_rdp.append([x, y])

            if len(points_for_rdp) >= 2:
                simplified_points_np = rdp(np.array(points_for_rdp), epsilon=self.simplification_tolerance)
                self.get_logger().info(f"Path: Original {len(points_for_rdp)}, Simplified {len(simplified_points_np)}")

                # Blue line
                line_marker = Marker()
                line_marker.header.frame_id = 'map'
                line_marker.header.stamp = self.get_clock().now().to_msg()
                line_marker.ns = 'simplified_contours'
                line_marker.id = simplified_marker_base_id
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                line_marker.pose.orientation.w = 1.0
                line_marker.scale.x = 0.008
                line_marker.color.r = 0.0
                line_marker.color.g = 0.0
                line_marker.color.b = 1.0
                line_marker.color.a = 1.0

                # Red spheres
                sphere_marker = Marker()
                sphere_marker.header.frame_id = 'map'
                sphere_marker.header.stamp = self.get_clock().now().to_msg()
                sphere_marker.ns = 'simplified_contours'
                sphere_marker.id = simplified_marker_base_id + 1
                sphere_marker.type = Marker.SPHERE_LIST
                sphere_marker.action = Marker.ADD
                sphere_marker.pose.orientation.w = 1.0
                sphere_marker.scale.x = 0.005
                sphere_marker.scale.y = 0.005
                sphere_marker.scale.z = 0.005
                sphere_marker.color.r = 1.0
                sphere_marker.color.g = 0.0
                sphere_marker.color.b = 0.0
                sphere_marker.color.a = 1.0

                for p in simplified_points_np:
                    pt = Point(x=p[0], y=p[1], z=0.0)
                    line_marker.points.append(pt)
                    sphere_marker.points.append(pt)

                if len(line_marker.points) > 1:
                    markers_simplified.markers.append(line_marker)
                    markers_simplified.markers.append(sphere_marker)
                    simplified_marker_base_id += 2
                else:
                    self.get_logger().warn(f"Skipping path with too few simplified points.")

        self.simplified_marker_array_pub.publish(markers_simplified)
        self.get_logger().info(f"Published {len(markers_simplified.markers)} markers to /simplified_svg_output.")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SVGWaypointPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
