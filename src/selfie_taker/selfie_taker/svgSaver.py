import numpy as np
import cv2
from rdp import rdp
from tqdm import tqdm
import svgwrite
import time


def save_as_svg(line_image, output_file, simplification_tolerance=2.0, svg_output_scale_factor=1.0):
    if line_image is None or line_image.size == 0:
        print("Error: Input line_image is empty or None.")
        return None

    start_time = time.time()
    height, width = line_image.shape
    svg_lines = []

    if line_image.dtype != np.uint8:
        if line_image.max() <= 1.0:
            line_image = (line_image * 255).astype(np.uint8)
        else:
            line_image = line_image.astype(np.uint8)

    _, binary_image = cv2.threshold(line_image, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    print("Processing contours and generating SVG lines...")

    base_pixel_to_meter_scale = 0.001  # 1 pixel = 1mm
    final_pixel_to_meter_scale = base_pixel_to_meter_scale * svg_output_scale_factor

    image_center_x_px = width / 2.0
    image_center_y_px = height / 2.0

    MIN_CONTOUR_LENGTH = 5

    for contour in tqdm(contours, desc="Simplifying and converting contours"):
        if len(contour) < MIN_CONTOUR_LENGTH:
            continue

        contour_2d = contour.reshape(-1, 2)
        simplified = rdp(contour_2d, epsilon=simplification_tolerance)

        if len(simplified) < 2:
            continue

        for i in range(len(simplified) - 1):
            x1_px, y1_px = simplified[i]
            x2_px, y2_px = simplified[i + 1]

            # Scale to meters
            x1_m = x1_px * final_pixel_to_meter_scale
            y1_m = y1_px * final_pixel_to_meter_scale
            x2_m = x2_px * final_pixel_to_meter_scale
            y2_m = y2_px * final_pixel_to_meter_scale

            # Translate and flip Y
            offset_x = image_center_x_px * final_pixel_to_meter_scale
            offset_y = image_center_y_px * final_pixel_to_meter_scale

            start = (x1_m - offset_x, -(y1_m - offset_y))
            end = (x2_m - offset_x, -(y2_m - offset_y))

            svg_lines.append((start, end))

    print(f"Generating SVG with {len(svg_lines)} line segments...")

    dwg = svgwrite.Drawing(output_file, size=(f"{width * final_pixel_to_meter_scale}m", f"{height * final_pixel_to_meter_scale}m"))
    for start, end in tqdm(svg_lines, desc="Writing SVG lines"):
        dwg.add(dwg.line(start=start, end=end, stroke='black', stroke_width=0.002))

    dwg.save()
    elapsed_time = time.time() - start_time

    print(f"SVG saved to {output_file} with {len(svg_lines)} lines.")
    print(f"Drawing size: {width * final_pixel_to_meter_scale:.3f}m x {height * final_pixel_to_meter_scale:.3f}m")
    print(f"Total processing time: {elapsed_time:.2f} seconds")

    return output_file
