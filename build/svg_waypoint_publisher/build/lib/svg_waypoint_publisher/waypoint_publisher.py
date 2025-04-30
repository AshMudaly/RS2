import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from svgpathtools import svg2paths2
import os
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SVGPosePublisher(Node):
    def __init__(self):
        super().__init__('svg_pose_publisher')
        marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        # Path to output.svg in ros2_ws
        self.svg_path = os.path.join('/home/ashmu/ros2_ws', 'output.svg')
        if not os.path.exists(self.svg_path):
            self.get_logger().error(f"SVG file not found at {self.svg_path}")
            sys.exit(1)

        self.all_markers = self.load_svg_paths()
        marker_pub.publish(self.all_markers)
        self.get_logger().info("Published all markers from SVG")

    def load_svg_paths(self):
        markers = MarkerArray()
        paths, attributes, svg_attributes = svg2paths2(self.svg_path)
        self.get_logger().info(f"SVG loaded with {len(paths)} paths")

        marker_id = 0
        for path in paths:
            for i in np.linspace(0, 1, 100):
                point = path.point(i)
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'svg_waypoints'
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point.real * 0.001
                marker.pose.position.y = -point.imag * 0.001
                marker.pose.position.z = 0.0
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0
                marker.color.a = 1.0
                markers.markers.append(marker)
                marker_id += 1
        return markers


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SVGPosePublisher()
        rclpy.spin(node)
        node.destroy_node()
    finally:
        rclpy.shutdown()
