from svgpathtools import Path, Line, wsvg
import numpy as np
import os  # Import the os module for path manipulation

def save_as_svg(line_image, output_file="output.svg"):  # Set a default output file name
    height, width = line_image.shape
    paths = []

    # Iterate through the binary image to find lines
    for y in range(height):
        for x in range(width):
            if line_image[y, x] > 0:  # If the pixel is part of an edge
                # Create a simple line for each edge pixel
                # (This can be extended to connect neighboring pixels into paths)
                start = complex(x, y)
                end = complex(x + 1, y)  # Horizontal line segment
                paths.append(Line(start, end))

    # Construct the full output path
    output_directory = "/home/ashmu/ros2_ws/RS2"
    full_output_path = os.path.join(output_directory, output_file)

    # Create the directory if it doesn't exist (optional, but good practice)
    os.makedirs(output_directory, exist_ok=True)

    # Create an SVG file from the paths at the specified location
    wsvg(paths, filename=full_output_path)
    print(f"SVG saved to {full_output_path}")
