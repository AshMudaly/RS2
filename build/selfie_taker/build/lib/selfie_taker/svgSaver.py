from svgpathtools import Path, Line, wsvg
import numpy as np

def save_as_svg(line_image, output_file):
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

    # Create an SVG file from the paths
    wsvg(paths, filename=output_file)
    print(f"SVG saved to {output_file}")