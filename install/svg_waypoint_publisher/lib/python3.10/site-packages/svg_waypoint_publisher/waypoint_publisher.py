import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from svgpathtools import svg2paths2, Line, Arc
import os
import cmath
import sys

class SVGPosePublisher(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisher')
        self.waypoint_pub = self.create_publisher(Marker, 'svg_waypoints', 10) # Changed to Marker

        # ... (rest of your __init__ code) ...

    def process_svg_and_publish(self):
        # ... (rest of your process_svg_and_publish code) ...
        marker_id = 0

        for path, attr in zip(paths, attributes):
            # ... (rest of the path processing) ...

            for segment in path:
                if isinstance(segment, Line):
                    start = segment.start
                    end = segment.end
                    self.publish_waypoint(start.real * 0.001, -start.imag * 0.001, marker_id)
                    marker_id += 1
                    self.publish_waypoint(end.real * 0.001, -end.imag * 0.001, marker_id)
                    marker_id += 1
                elif isinstance(segment, Arc):
                    n_points = 20
                    for i in range(n_points + 1):
                        t = i / n_points
                        point = segment.point(t)
                        self.publish_waypoint(point.real * 0.001, -point.imag * 0.001, marker_id)
                        marker_id += 1
                # ... (rest of segment processing) ...

    def publish_waypoint(self, x, y, marker_id):
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
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.waypoint_pub.publish(marker)
        self.get_logger().info(f"Published waypoint: x={x:.3f}, y={y:.3f}, id={marker_id}")

    # Removed create_waypoint_marker and waypoint_marker_array_pub

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SVGPosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()