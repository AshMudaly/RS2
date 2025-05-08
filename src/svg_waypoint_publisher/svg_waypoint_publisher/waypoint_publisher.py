import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import tf_transformations
from svgpathtools import svg2paths2, Line, Arc  # Import necessary classes
import os
import cmath
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
from RS2_SelfieRobot.Image_Processor.Processor import ImageProcessor

class SVGPosePublisher(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisherS')  # Node name is 'svg_waypoint_publisher'
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.waypoint_marker_array_pub = self.create_publisher(MarkerArray, 'svg_waypoints', 10)

        # Declare and get the SVG file path
        self.declare_parameter('svg_file', 'output.svg')  # Default value is 'shape.svg'
        self.svg_file = self.get_parameter('svg_file').get_parameter_value().string_value
        self.get_logger().info(f"Using SVG file: {self.svg_file}")

        # Construct the full path to the SVG file
        self.svg_path = os.path.join('/home/ashmu/ros2_ws', self.svg_file)

        if not os.path.exists(self.svg_path):
            self.get_logger().error(f"SVG file not found at {self.svg_path}")
            sys.exit(1)

        self.process_svg_and_publish()

    def process_svg_and_publish(self):
        markers = MarkerArray()
        waypoint_markers = MarkerArray()
        paths, attributes, svg_attributes = svg2paths2(self.svg_path)
        line_marker_id = 0
        waypoint_marker_id = 0

        for path, attr in zip(paths, attributes):
            color = attr.get('stroke', '').lower()
            self.get_logger().info(f"Path color: {color}")
            if color != '#000000' and color != 'black':
                continue

            line_marker = Marker()
            line_marker.header.frame_id = 'map'
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'svg_paths'
            line_marker.id = line_marker_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.005
            line_marker.color.r = 0.0
            line_marker.color.g = 0.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_points = []

            for segment in path:
                if isinstance(segment, Line):
                    start = segment.start
                    end = segment.end
                    line_points.append(Point(x=start.real * 0.001, y=-start.imag * 0.001, z=0.0))
                    line_points.append(Point(x=end.real * 0.001, y=-end.imag * 0.001, z=0.0))

                    # Publish start and end points as waypoints
                    waypoint_marker = self.create_waypoint_marker(start.real * 0.001, -start.imag * 0.001, waypoint_marker_id)
                    waypoint_markers.markers.append(waypoint_marker)
                    waypoint_marker_id += 1
                    waypoint_marker = self.create_waypoint_marker(end.real * 0.001, -end.imag * 0.001, waypoint_marker_id)
                    waypoint_markers.markers.append(waypoint_marker)
                    waypoint_marker_id += 1

                elif isinstance(segment, Arc):
                    n_points = 20
                    for i in range(n_points + 1):
                        t = i / n_points
                        point = segment.point(t)
                        line_points.append(Point(x=point.real * 0.001, y=-point.imag * 0.001, z=0.0))

                        # Publish each sampled point as a waypoint
                        waypoint_marker = self.create_waypoint_marker(point.real * 0.001, -point.imag * 0.001, waypoint_marker_id)
                        waypoint_markers.markers.append(waypoint_marker)
                        waypoint_marker_id += 1
                else:
                    self.get_logger().warn(f"Unsupported segment type: {type(segment)}")

            line_marker.points = line_points
            if len(line_points) > 1:
                markers.markers.append(line_marker)
                line_marker_id += 1
            else:
                self.get_logger().warn(f"Skipping line marker with less than 2 points. Path: {path}")

        self.marker_array_pub.publish(markers)
        self.get_logger().info(f"Published {len(markers.markers)} markers representing the SVG paths.")

        self.waypoint_marker_array_pub.publish(waypoint_markers)
        self.get_logger().info(f"Published {len(waypoint_markers.markers)} waypoint markers from the SVG.")

    def create_waypoint_marker(self, x, y, marker_id):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'svg_waypoints'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01  # Adjust scale as needed
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SVGPosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node')
    finally:
        if 'node' in locals():  # Ensure node is defined before destroying
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()