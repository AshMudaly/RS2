import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header, ColorRGBA
from svgpathtools import svg2paths
from rdp import rdp
import numpy as np
import time
import os

class SVGWaypointPublisherNode(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisher')

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/simplified_svg_output', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/move_group/goal', 10)

        # Load and process SVG
        self.svg_path = os.path.expanduser('~/ros2_ws/RS2/output.svg')

        if not os.path.exists(self.svg_path):
            self.get_logger().error(f'SVG file not found: {self.svg_path}')
            return
        
        self.publish_markers_and_poses()

    def publish_markers_and_poses(self):
        marker_array = MarkerArray()
        marker_id = 0

        paths, _ = svg2paths(self.svg_path)
        for path in paths:
            line_points = []
            for segment in path:
                for t in np.linspace(0, 1, num=20):
                    point = segment.point(t)
                    line_points.append([point.real / 100.0, point.imag / 100.0])  # scale to meters

            simplified_points = rdp(line_points, epsilon=0.005)

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'svg_path'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.002  # line width
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

            for point in simplified_points:
                marker.points.append(Point(x=point[0], y=point[1], z=0.0))

                # Publish PoseStamped to the robot
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.1  # safe height above table
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0  # no rotation

                self.pose_pub.publish(pose)
                self.get_logger().info(f'Published pose: x={point[0]:.3f}, y={point[1]:.3f}, z=0.1')

                time.sleep(1.0)  # Delay for robot to execute

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info('Published all markers and poses.')

def main(args=None):
    rclpy.init(args=args)
    node = SVGWaypointPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
