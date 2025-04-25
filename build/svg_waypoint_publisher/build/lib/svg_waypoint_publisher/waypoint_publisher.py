import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from svgpathtools import svg2paths
import numpy as np

class SVGWaypointPublisher(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisher')

        self.publisher_ = self.create_publisher(Marker, 'svg_waypoints', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "svg_points"
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

        self.load_svg('output.svg')

    def load_svg(self, filename):
        paths, _ = svg2paths(filename)
        for path in paths:
            for i in np.linspace(0, 1, 100):
                point = path.point(i)
                p = Point()
                p.x = point.real * 0.001  # Scale to meters
                p.y = point.imag * 0.001
                p.z = 0.0
                self.marker.points.append(p)

    def publish_waypoints(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.marker)
        self.get_logger().info("Published waypoints to RViz")

def main(args=None):
    rclpy.init(args=args)
    node = SVGWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
