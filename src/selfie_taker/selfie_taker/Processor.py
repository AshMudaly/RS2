import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import rembg
import time
from . import lineGenerator
from . import faceDetector
from . import svgSaver
import os
from tqdm import tqdm # Import tqdm here for potential top-level use

class SelfieProcessorNode(Node):
    def __init__(self):
        super().__init__('selfie_processor_node')
        self.get_logger().info('Selfie processor node has started!')

        self.declare_parameter('output_svg_path', '/home/ashmu/ros2_ws/RS2/output.svg')
        self.output_svg_path = self.get_parameter('output_svg_path').get_parameter_value().string_value
        self.get_logger().info(f"Output SVG will be saved to: {self.output_svg_path}")

        self.declare_parameter('svg_output_simplification_tolerance', 1.5)
        self.svg_output_simplification_tolerance = self.get_parameter('svg_output_simplification_tolerance').get_parameter_value().double_value
        self.get_logger().info(f"Using SVG output simplification tolerance: {self.svg_output_simplification_tolerance}")

        self.declare_parameter('svg_output_scale_factor', 1.5)
        self.svg_output_scale_factor = self.get_parameter('svg_output_scale_factor').get_parameter_value().double_value
        self.get_logger().info(f"Using SVG output scale factor: {self.svg_output_scale_factor}")

        # Call the processing function once on startup
        self.process_image_and_save_svg()

    def process_image_and_save_svg(self):
        self.get_logger().info('Starting image processing pipeline...')

        if os.path.exists(self.output_svg_path):
            try:
                os.remove(self.output_svg_path)
                self.get_logger().info(f'Existing SVG file deleted: {self.output_svg_path}')
            except OSError as e:
                self.get_logger().error(f'Error deleting existing SVG file: {e}')

        # Step 1: Face Detection & Cropping
        self.get_logger().info('Detecting and cropping face...')
        # If faceDetector has an internal loop or can report progress, you'd integrate it here.
        # For now, it's just a blocking call.
        cropped_face = faceDetector.detect_and_crop_face()

        if cropped_face is not None:
            self.get_logger().info('Face detected. Generating line image...')
            # Step 2: Line Generation
            # If lineGenerator has an internal loop or can report progress, integrate it here.
            # For now, it's just a blocking call.
            lineImage = lineGenerator.lineGenerator(cropped_face)

            if lineImage is not None:
                self.get_logger().info('Line image generated. Saving SVG...')
                # Step 3: SVG Saving (which now has its own tqdm bar internally)
                svg_file_path = svgSaver.save_as_svg(
                    lineImage,
                    self.output_svg_path,
                    simplification_tolerance=self.svg_output_simplification_tolerance,
                    svg_output_scale_factor=self.svg_output_scale_factor
                )
                self.get_logger().info(f'SVG processing complete: {svg_file_path}')
            else:
                self.get_logger().warn('Failed to generate line image, SVG not saved.')
        else:
            self.get_logger().warn('No face detected, SVG not saved.')

def main(args=None):
    rclpy.init(args=args)
    selfie_processor_node = SelfieProcessorNode()
    rclpy.spin_once(selfie_processor_node)
    selfie_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()