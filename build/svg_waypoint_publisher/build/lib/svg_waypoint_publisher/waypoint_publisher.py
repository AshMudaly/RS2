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
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
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
        markers_original = MarkerArray()
        markers_simplified = MarkerArray()

        paths, attributes, svg_attributes = svg2paths2(self.svg_path)
        original_line_marker_id = 0
        simplified_marker_base_id = 0
        N_POINTS_PER_ARC_SEGMENT = 50

        # Pass 1: Gather all raw SVG points to find the center
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

        # Pass 2: Create markers using centered coordinates
        for path, attr in zip(paths, attributes):
            color = attr.get('stroke', '').lower()
            if color != '#000000' and color != 'black':
                continue

            original_line_points_list = []
            points_for_rdp = []

            for segment in path:
                if isinstance(segment, Line):
                    x1 = segment.start.real - center[0]
                    y1 = -segment.start.imag - center[1]
                    x2 = segment.end.real - center[0]
                    y2 = -segment.end.imag - center[1]

                    original_line_points_list.append(Point(x=x1, y=y1, z=0.0))
                    original_line_points_list.append(Point(x=x2, y=y2, z=0.0))

                    points_for_rdp.append([x1, y1])
                    points_for_rdp.append([x2, y2])

                elif isinstance(segment, (Arc, CubicBezier, QuadraticBezier)):
                    for i in range(N_POINTS_PER_ARC_SEGMENT + 1):
                        t = i / N_POINTS_PER_ARC_SEGMENT
                        point_complex = segment.point(t)
                        x = point_complex.real - center[0]
                        y = -point_complex.imag - center[1]

                        point_msg = Point(x=x, y=y, z=0.0)
                        original_line_points_list.append(point_msg)
                        points_for_rdp.append([x, y])
                else:
                    self.get_logger().warn(f"Unsupported segment type: {type(segment)} for path {original_line_marker_id}")

            if len(original_line_points_list) > 1:
                line_marker_original = Marker()
                line_marker_original.header.frame_id = 'map'
                line_marker_original.header.stamp = self.get_clock().now().to_msg()
                line_marker_original.ns = 'svg_paths_original'
                line_marker_original.id = original_line_marker_id
                line_marker_original.type = Marker.LINE_STRIP
                line_marker_original.action = Marker.ADD
                line_marker_original.pose.orientation.w = 1.0
                line_marker_original.scale.x = 0.005
                line_marker_original.color.r = 0.0
                line_marker_original.color.g = 0.0
                line_marker_original.color.b = 0.0
                line_marker_original.color.a = 1.0
                line_marker_original.points = original_line_points_list
                markers_original.markers.append(line_marker_original)
                original_line_marker_id += 1
            else:
                self.get_logger().warn(f"Skipping original line marker with less than 2 points. Path ID: {original_line_marker_id}")

            if len(points_for_rdp) >= 2:
                points_for_rdp_np = np.array(points_for_rdp)
                simplified_points_np = rdp(points_for_rdp_np, epsilon=self.simplification_tolerance)

                self.get_logger().info(f"Path {original_line_marker_id-1}: Original points: {len(points_for_rdp_np)}, Simplified points: {len(simplified_points_np)}")

                simplified_line_marker = Marker()
                simplified_line_marker.header.frame_id = 'map'
                simplified_line_marker.header.stamp = self.get_clock().now().to_msg()
                simplified_line_marker.ns = 'simplified_contours'
                simplified_line_marker.id = simplified_marker_base_id
                simplified_line_marker.type = Marker.LINE_STRIP
                simplified_line_marker.action = Marker.ADD
                simplified_line_marker.pose.orientation.w = 1.0
                simplified_line_marker.scale.x = 0.008
                simplified_line_marker.color.r = 0.0
                simplified_line_marker.color.g = 0.0
                simplified_line_marker.color.b = 1.0
                simplified_line_marker.color.a = 1.0

                simplified_points_marker = Marker()
                simplified_points_marker.header.frame_id = 'map'
                simplified_points_marker.header.stamp = self.get_clock().now().to_msg()
                simplified_points_marker.ns = 'simplified_contours'
                simplified_points_marker.id = simplified_marker_base_id + 1
                simplified_points_marker.type = Marker.SPHERE_LIST
                simplified_points_marker.action = Marker.ADD
                simplified_points_marker.pose.orientation.w = 1.0
                simplified_points_marker.scale.x = 0.005
                simplified_points_marker.scale.y = 0.005
                simplified_points_marker.scale.z = 0.005
                simplified_points_marker.color.r = 1.0
                simplified_points_marker.color.g = 0.0
                simplified_points_marker.color.b = 0.0
                simplified_points_marker.color.a = 1.0

                for p_rdp in simplified_points_np:
                    point_msg = Point(x=p_rdp[0], y=p_rdp[1], z=0.0)
                    simplified_line_marker.points.append(point_msg)
                    simplified_points_marker.points.append(point_msg)

                if len(simplified_line_marker.points) > 1:
                    markers_simplified.markers.append(simplified_line_marker)
                    markers_simplified.markers.append(simplified_points_marker)
                    simplified_marker_base_id += 2
                else:
                    self.get_logger().warn(f"Skipping simplified markers with less than 2 points for path {original_line_marker_id-1}.")

        self.marker_array_pub.publish(markers_original)
        self.get_logger().info(f"Published {len(markers_original.markers)} original path markers to /visualization_marker_array.")

        self.simplified_marker_array_pub.publish(markers_simplified)
        self.get_logger().info(f"Published {len(markers_simplified.markers)} simplified line/point markers to /simplified_svg_output.")

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
