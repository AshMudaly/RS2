import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from svgpathtools import svg2paths
import numpy as np


class SVGWaypointPublisher(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisher')

        # Publisher for visualizing waypoints
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load the SVG waypoints
        self.waypoints = self.load_svg_points('/home/ashmu/ros2_ws/RS2/src/svg_generation/output.svg')
        self.index = 0

        # Setup Marker
        self.marker = Marker()
        self.marker.header.frame_id = "world"  # Set your fixed frame (base_link, world, etc.)
        self.marker.header.stamp = rclpy.time.Time()
        self.marker.ns = "waypoints"
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05  # Point size
        self.marker.scale.y = 0.05  # Point size
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

    def load_svg_points(self, svg_path):
        """Extract points from the SVG."""
        paths, _ = svg2paths(svg_path)
        points = []
        for path in paths:
            for i in np.linspace(0, 1, num=50):  # Adjust sampling density
                pt = path.point(i)
                points.append((pt.real, pt.imag))
        return points

    def timer_callback(self):
        """Update the marker with the next waypoint and publish it."""
        if self.index < len(self.waypoints):
            x, y = self.waypoints[self.index]
            self.marker.points.append({'x': x, 'y': y, 'z': 0.0})

            # Publish the marker
            self.publisher.publish(self.marker)
            self.index += 1

        if self.index >= len(self.waypoints):
            self.index = 0  # Loop the points

def main():
    rclpy.init()

    # Create the SVG waypoint publisher node
    svg_publisher = SVGWaypointPublisher()

    try:
        rclpy.spin(svg_publisher)
    except KeyboardInterrupt:
        pass

    svg_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
