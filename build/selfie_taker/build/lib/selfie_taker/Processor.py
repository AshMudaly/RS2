import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import rembg
import time
from svgpathtools import Path, Line, wsvg
from . import lineGenerator  # Use relative imports within the package
from . import faceDetector
from . import svgSaver
import os

class SelfieProcessorNode(Node):
    def __init__(self):
        super().__init__('selfie_processor_node')
        self.get_logger().info('Selfie processor node has started!')
        self.output_svg_path = '/home/ashmu/ros2_ws/RS2/output.svg'
        self.process_image_and_save_svg()  # Call the processing function once on startup

    def process_image_and_save_svg(self):
        self.get_logger().info('Processing image and saving SVG...')
        # Check if an existing SVG file exists and delete it
        if os.path.exists(self.output_svg_path):
            try:
                os.remove(self.output_svg_path)
                self.get_logger().info(f'Existing SVG file deleted: {self.output_svg_path}')
            except OSError as e:
                self.get_logger().error(f'Error deleting existing SVG file: {e}')

        cropped_face = faceDetector.detect_and_crop_face()
        if cropped_face is not None:
            lineImage = lineGenerator.lineGenerator(cropped_face)
            svg_file_path = svgSaver.save_as_svg(lineImage, "output.svg")
            self.get_logger().info(f'SVG saved to: {svg_file_path}')
        else:
            self.get_logger().warn('No face detected, SVG not saved.')

def main(args=None):
    rclpy.init(args=args)
    selfie_processor_node = SelfieProcessorNode()
    rclpy.spin_once(selfie_processor_node) # Run the node's tasks once and exit
    selfie_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()