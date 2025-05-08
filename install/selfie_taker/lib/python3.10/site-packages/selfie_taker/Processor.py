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

class SelfieProcessorNode(Node):
    def __init__(self):
        super().__init__('selfie_processor_node')
        self.get_logger().info('Selfie processor node has started!')
        self.process_image_and_save_svg()  # Call the processing function once on startup

    def process_image_and_save_svg(self):
        self.get_logger().info('Processing image and saving SVG...')
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
