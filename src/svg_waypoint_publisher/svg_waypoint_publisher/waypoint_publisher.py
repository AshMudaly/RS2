import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import tf_transformations
from svgpathtools import svg2paths2, Line, Arc
import os
import cmath
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time  # Import time for debugging
import sys  # Import sys


class SVGPosePublisher(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisher')
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher_ = self.create_publisher(PoseStamped, 'ur3e_target_pose',
                                                qos_profile=pose_qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.path_pub_ = self.create_publisher(MarkerArray, 'svg_path_marker_array', 10)  # New publisher for the SVG path

        # Use a parameter for the SVG file path (more flexible)
        self.declare_parameter('svg_file', 'output.svg')  # Default value
        self.svg_file = self.get_parameter('svg_file')._value
        self.get_logger().info(f"Using SVG file: {self.svg_file}")

        # Construct the SVG path.
        self.svg_path = os.path.join('/home/ashmu/ros2_ws', self.svg_file)

        if not os.path.exists(self.svg_path):
            self.get_logger().error(f"SVG file not found at {self.svg_path}")
            sys.exit(1)

        self.poses = self.load_svg_black_paths(self.svg_path)
        self.all_markers = self.create_markers(self.poses)
        self.publish_all_markers()
        self.path_markers = self.create_path_markers(self.svg_path)  # Create path markers
        self.publish_path_markers() # Publish path markers
        self.timer = self.create_timer(0.1, self.publish_next_pose)
        self.index = 0
        self.last_published_time = time.time()

    def load_svg_black_paths(self, filename):
        poses = []
        paths, attributes, svg_attributes = svg2paths2(filename)
        self.get_logger().info(f"SVG loaded with {len(paths)} paths")
        distance_threshold = 0.01
        sampling_distance = 0.1  # Increased sampling distance for fewer waypoints
        last_pose = None

        for path, attr in zip(paths, attributes):
            color = attr.get('stroke', '').lower()
            if color != '#000000' and color != 'black':
                continue
            path_length = path.length()
            num_samples = int(path_length / sampling_distance) + 1

            for i in np.linspace(0, 1, num_samples):
                point = path.point(i)
                if last_pose is not None:
                    distance = np.sqrt(
                        (point.real - last_pose[0]) ** 2 +
                        (point.imag - last_pose[1]) ** 2
                    )
                    if distance < distance_threshold:
                        continue
                tangent = path.derivative(i)
                angle = cmath.phase(tangent)
                roll = 0.0
                pitch = 0.0
                yaw = angle
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose.position.x = point.real * 0.001
                pose_stamped.pose.position.y = -point.imag * 0.001
                pose_stamped.pose.position.z = 0.0
                q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]
                poses.append(pose_stamped)
                last_pose = (point.real, point.imag)
        self.get_logger().info(f"Generated {len(poses)} poses from black paths")
        return poses

    def create_markers(self, poses):
        markers = []
        for i, pose in enumerate(poses):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 1.0
            markers.append(marker)
        return markers

    def publish_all_markers(self):
        marker_array = MarkerArray()
        marker_array.markers = self.all_markers
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published all {len(self.all_markers)} markers")

    def create_path_markers(self, svg_file):
        """
        Creates a MarkerArray representing the SVG path for visualization in RViz.
        """
        markers = MarkerArray()
        paths, attributes, svg_attributes = svg2paths2(svg_file)
        marker_id = 0

        for path, attr in zip(paths, attributes):
            # Create a line strip marker for each path.
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'svg_path'
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0  # Default orientation
            marker.scale.x = 0.005  # Line width
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            points = []

            # Iterate through the segments of the path.
            for segment in path:
                if isinstance(segment, Line):
                    start = segment.start
                    end = segment.end
                    points.append(Point(x=start.real * 0.001, y=-start.imag * 0.001, z=0.0))
                    points.append(Point(x=end.real * 0.001, y=-end.imag * 0.001, z=0.0))
                elif isinstance(segment, Arc):
                    # Subsample the arc to get points along its curve
                    n_points = 10  # Adjust for desired smoothness
                    for i in range(n_points + 1):
                        t = i / n_points
                        point = segment.point(t)
                        points.append(Point(x=point.real * 0.001, y=-point.imag * 0.001, z=0.0))
                else:
                    # Handle other segment types (e.g., CubicBezier, QuadraticBezier) as needed
                    self.get_logger().warn(f"Unsupported segment type: {type(segment)}")

            marker.points = points
            if len(points) > 1:  # Ensure the marker has at least two points
                markers.markers.append(marker)
                marker_id += 1
            else:
                self.get_logger().warn(f"Skipping marker with less than 2 points. Path: {path}")
        return markers
    def publish_path_markers(self):
        """
        Publishes the MarkerArray representing the SVG path.
        """
        self.path_pub_.publish(self.path_markers)
        self.get_logger().info("Published SVG path markers")

    def publish_next_pose(self):
        if self.index < len(self.poses):
            pose_stamped = self.poses[self.index]
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(pose_stamped)
            current_time = time.time()
            if current_time - self.last_published_time >= 0.1:
                self.get_logger().info(f'Published pose {self.index + 1}/{len(self.poses)}')
                self.last_published_time = current_time
            self.index += 1
        else:
            self.get_logger().info("All poses published.")
            self.index = 0

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SVGPosePublisher()
        rclpy.spin(node)
        node.destroy_node()
    finally:
        rclpy.shutdown()

