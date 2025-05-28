import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from svgpathtools import svg2paths
from rdp import rdp
import numpy as np
import os
import cv2

# ... (Your SVGWaypointPublisher class from the previous version here) ...
# Make sure the SVGWaypointPublisher class is defined before main()
class SVGWaypointPublisher(Node):
    def __init__(self):
        super().__init__('svg_waypoint_publisher')
        self.pose_array_pub = self.create_publisher(PoseArray, '/svg_waypoints/poses', 10)
        self.svg_path = '/home/alec/ros2_ws/src/RS2/output.svg' # Make sure this path is correct
        self.image_window_name = 'SVG Visualization' # Ensure this is consistent

        if not os.path.exists(self.svg_path):
            self.get_logger().error(f"SVG file not found: {self.svg_path}")
            raise RuntimeError(f"SVG file not found: {self.svg_path}")

        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = 'map'
        self.processed_points = []

        self.workspace_x_min = -0.125
        self.workspace_x_max = 0.125
        self.workspace_y_min = 0.310
        self.workspace_y_max = 0.490
        self.workspace_width = self.workspace_x_max - self.workspace_x_min
        self.workspace_height = self.workspace_y_max - self.workspace_y_min
        self.workspace_aspect = self.workspace_width / self.workspace_height

        self.image_scale_factor = 2000
        self.image_width = int(self.workspace_width * self.image_scale_factor)
        self.image_height = int(self.workspace_height * self.image_scale_factor)
        # Ensure image dimensions are positive
        if self.image_width <= 0 or self.image_height <= 0:
            err_msg = f"Calculated image dimensions are invalid: {self.image_width}x{self.image_height}"
            self.get_logger().error(err_msg)
            raise ValueError(err_msg)

        self.bg_color = (255, 255, 255)
        self.line_color = (0, 0, 255)

        self.process_svg()
        if rclpy.ok():
            self.publish_poses()
            self.get_logger().info("SVG processing and initial publish complete. Node is alive to keep visualization.")

    def process_svg(self):
        self.get_logger().info(f"Processing SVG file: {self.svg_path}")
        paths, _ = svg2paths(self.svg_path)
        all_points = []
        for path in paths:
            points = []
            for segment in path:
                for t in np.linspace(0, 1, num=20):
                    pt = segment.point(t)
                    points.append([pt.real, pt.imag])
            if points:
                all_points.append(np.array(points))

        # Create a blank image for visualization (do this early, so window name is known)
        # Ensure image dimensions are valid before this call
        if self.image_height <=0 or self.image_width <=0:
             self.get_logger().error(f"Cannot create image with non-positive dimensions: {self.image_height}x{self.image_width}")
             # Potentially show a default small blank image or error message image
             # For now, let's ensure this is caught earlier or handled.
             # If process_svg is called, image_height/width should be > 0 from __init__ check
             pass # Assuming dimensions are valid due to check in __init__
        
        image = np.full((self.image_height, self.image_width, 3), self.bg_color, dtype=np.uint8)


        if not all_points:
            self.get_logger().warn("No points extracted from SVG. Displaying blank workspace.")
            cv2.imshow(self.image_window_name, image) # Show blank image
            cv2.waitKey(1) # Crucial call
            return

        all_points_np = np.vstack(all_points)
        self.min_xy = all_points_np.min(axis=0)
        max_xy = all_points_np.max(axis=0)
        self.svg_width = max_xy[0] - self.min_xy[0]
        self.svg_height = max_xy[1] - self.min_xy[1]

        if self.svg_width <= 0 or self.svg_height <= 0: # Simplified check for non-zero area
            if self.svg_width == 0 and self.svg_height == 0:
                self.get_logger().warn("SVG content has no width or height (single point). Using scale 1.0.")
                self.scale = 1.0
            elif self.svg_width <= 0 : # effectively a vertical line or point
                 self.scale = self.workspace_height / self.svg_height if self.svg_height > 0 else 1.0
            elif self.svg_height <= 0 : # effectively a horizontal line or point
                 self.scale = self.workspace_width / self.svg_width if self.svg_width > 0 else 1.0
            else: # Should not be reached if previous conditions are exhaustive
                self.scale = 1.0
        else:
            svg_aspect = self.svg_width / self.svg_height
            if svg_aspect > self.workspace_aspect:
                self.scale = self.workspace_width / self.svg_width
            else:
                self.scale = self.workspace_height / self.svg_height
        
        if self.scale <= 0: # Safety check for scale
            self.get_logger().warn(f"Calculated scale is {self.scale}. Resetting to 1.0. This might indicate issues with SVG or workspace dimensions.")
            self.scale = 1.0


        self.pose_array.poses = []
        self.processed_points = []

        Z_DRAW = 0.157
        Z_TRAVEL = 0.1625

        for i, points_np in enumerate(all_points):
            if points_np.shape[0] < 75:
                self.get_logger().warn(f"Skipping contour {i} with less than 2 points before RDP.")
                continue
            simplified_raw = rdp(points_np.tolist(), epsilon=0.005)
            if not simplified_raw or len(simplified_raw) < 1:
                self.get_logger().warn(f"Skipping contour {i} as it resulted in no points after RDP.")
                continue

            scaled_simplified = []
            for pt_idx, pt in enumerate(simplified_raw):
                scaled_pt_real = (pt[0] - self.min_xy[0]) * self.scale + self.workspace_x_min
                scaled_pt_imag = (pt[1] - self.min_xy[1]) * self.scale + self.workspace_y_min
                scaled_pt_real += (self.workspace_width - self.svg_width * self.scale) / 2.0
                scaled_pt_imag += (self.workspace_height - self.svg_height * self.scale) / 2.0
                scaled_simplified.append(np.array([scaled_pt_real, scaled_pt_imag]))

            if not scaled_simplified: continue

            if len(scaled_simplified) > 1:
                for j in range(len(scaled_simplified) - 1):
                    pt1_img = self._to_image_coords(scaled_simplified[j])
                    pt2_img = self._to_image_coords(scaled_simplified[j+1])
                    cv2.line(image, pt1_img, pt2_img, self.line_color, 2)
            elif len(scaled_simplified) == 1:
                 pt_img = self._to_image_coords(scaled_simplified[0])
                 cv2.circle(image, pt_img, 3, self.line_color, -1)

            current_path_first_point = scaled_simplified[0]
            if self.pose_array.poses:
                self.pose_array.poses.append(self._create_pose(current_path_first_point[0], current_path_first_point[1], Z_TRAVEL))
            self.pose_array.poses.append(self._create_pose(current_path_first_point[0], current_path_first_point[1], Z_DRAW))

            for pt_idx in range(len(scaled_simplified)):
                if pt_idx > 0:
                     self.pose_array.poses.append(self._create_pose(scaled_simplified[pt_idx][0], scaled_simplified[pt_idx][1], Z_DRAW))

            if i < len(all_points) - 1 and len(scaled_simplified) > 0:
                last_pt_in_contour = scaled_simplified[-1]
                self.pose_array.poses.append(self._create_pose(last_pt_in_contour[0], last_pt_in_contour[1], Z_TRAVEL))
            self.processed_points.extend(scaled_simplified)

        cv2.imshow(self.image_window_name, image)
        cv2.waitKey(30) # Give it a bit more time to render, e.g., 30ms

        self.get_logger().info(f"Processed and scaled {len(self.pose_array.poses)} waypoints.")

    def _create_pose(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 1.0 # Assuming tool points down, often needs rotation e.g. around Y
        pose.orientation.y = 0.0 # For identity (no rotation from frame): w=1, x=y=z=0
        pose.orientation.z = 0.0 # For 180 deg around X (often for cameras): x=1, y=z=w=0
        pose.orientation.w = 0.0 # Default to identity quaternion (no rotation relative to frame_id)
                                 # Your previous was x=1, w=0. This implies a 180 deg rotation around X.
                                 # Identity (0,0,0,1) or a specific rotation for "pen down" is common.
                                 # I'll use identity here, adjust if needed.
        return pose

    def _to_image_coords(self, point_in_workspace):
        point_in_workspace = np.asarray(point_in_workspace)
        # Check for NaN or Inf in input coordinates, which can cause crashes
        if np.any(np.isnan(point_in_workspace)) or np.any(np.isinf(point_in_workspace)):
            self.get_logger().warn(f"Invalid coordinates in _to_image_coords: {point_in_workspace}. Using (0,0).")
            return (0,0)

        img_x = ((point_in_workspace[0] - self.workspace_x_min) / self.workspace_width) * self.image_width
        img_y = self.image_height - ((point_in_workspace[1] - self.workspace_y_min) / self.workspace_height) * self.image_height
        
        # Cast to int AFTER scaling and BEFORE clamping, if numbers are huge, int() can overflow before clamping
        # However, for image coordinates, they should be within a reasonable float range before int casting
        img_x_int = int(img_x)
        img_y_int = int(img_y)

        img_x_clamped = max(0, min(img_x_int, self.image_width - 1))
        img_y_clamped = max(0, min(img_y_int, self.image_height - 1))
        return (img_x_clamped, img_y_clamped)

    def publish_poses(self):
        if not self.pose_array.poses:
            self.get_logger().warn("No poses to publish.")
            return
        self.pose_array.header.stamp = self.get_clock().now().to_msg()
        self.pose_array_pub.publish(self.pose_array)
        self.get_logger().info(f"Published {len(self.pose_array.poses)} poses once.")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SVGWaypointPublisher() # This calls process_svg which calls imshow

        if rclpy.ok() and node:
            node.get_logger().info("Node initialized. Entering spin/visualization loop.")
            while rclpy.ok():
                # Process a few ROS events. Non-blocking.
                rclpy.spin_once(node, timeout_sec=0.01)  # 10ms timeout for ROS events

                # Process OpenCV window events.
                # This is crucial for the window to be responsive and not crash.
                key = cv2.waitKey(20)  # Wait for 20ms for a key press.
                                       # Adjust delay as needed. Too small can burn CPU.
                                       # Too large makes window less responsive.

                if key == 27:  # Check for ESC key (ASCII 27)
                    node.get_logger().info("ESC key pressed. Shutting down.")
                    break  # Exit the loop, will lead to finally block

                # Check if the window was closed using the 'X' button
                # This check can be unreliable on some systems/backends (e.g., Wayland).
                window_visible = True
                try:
                    # Check if window still exists and is visible
                    # WND_PROP_VISIBLE might return 0 if minimized, use WND_PROP_AUTOSIZE as a proxy for existence
                    # If getWindowProperty raises an error, the window is likely gone.
                    if cv2.getWindowProperty(node.image_window_name, cv2.WND_PROP_VISIBLE) < 1:
                         # Check if it's just minimized, if WND_PROP_AUTOSIZE is also gone, it's likely closed
                         if cv2.getWindowProperty(node.image_window_name, cv2.WND_PROP_AUTOSIZE) < 0 : # Often -1 if destroyed
                            node.get_logger().info("Visualization window closed by user (AUTOSIZE check). Shutting down.")
                            window_visible = False
                         # else: still visible or minimized, continue
                except cv2.error:
                    # This error occurs if the window has been destroyed.
                    node.get_logger().info("Visualization window destroyed (cv2.error). Shutting down.")
                    window_visible = False
                
                if not window_visible:
                    break

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('KeyboardInterrupt, shutting down.')
    except RuntimeError as e: # Catch errors like SVG not found from __init__
        if node: # Node might exist but failed during its setup
            node.get_logger().error(f"Runtime error during execution: {e}")
        else: # Error happened before node object could be assigned
            print(f"Critical runtime error (node not fully initialized): {e}")
    except ValueError as e: # Catch errors like invalid image dimensions
        if node:
            node.get_logger().error(f"ValueError: {e}")
        else:
            print(f"Critical ValueError: {e}")
    except Exception as e: # Catch any other unexpected errors
        if node:
            node.get_logger().error(f"An unexpected error occurred: {e}", exc_info=True)
        else:
            print(f"An unexpected error occurred (node not initialized): {e}")
    finally:
        if node:
            node.get_logger().info("Cleaning up node...")
            node.destroy_node()
        # cv2.destroyAllWindows() should always be called to be safe
        # It's idempotent if windows are already closed.
        cv2.destroyAllWindows()
        if rclpy.ok(): # Check if rclpy is still initialized before shutting down
            rclpy.shutdown()
        print("ROS2 shutdown sequence complete and OpenCV windows closed.")

if __name__ == '__main__':
    main()