from svgpathtools import Path, Line, wsvg
import numpy as np
import cv2
from rdp import rdp
from tqdm import tqdm  # Import tqdm


def save_as_svg(line_image, output_file, simplification_tolerance=1.0, svg_output_scale_factor=1.0):
    if line_image is None or line_image.size == 0:
        print("Error: Input line_image is empty or None.")
        return None

    height, width = line_image.shape
    svg_paths = []

    # Step 1: Normalize the image
    if line_image.dtype != np.uint8:
        if line_image.max() <= 1.0:
            line_image = (line_image * 255).astype(np.uint8)
        else:
            line_image = line_image.astype(np.uint8)

    # Step 2: Binarize and find contours
    _, binary_image = cv2.threshold(line_image, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    print("Processing contours and generating SVG paths...")

    base_pixel_to_meter_scale = 0.001  # 1 pixel = 1mm
    final_pixel_to_meter_scale = base_pixel_to_meter_scale * svg_output_scale_factor

    image_center_x_px = width / 2.0
    image_center_y_px = height / 2.0

    for contour in tqdm(contours, desc="SVG Generation"):
        if len(contour) < 2:
            continue

        contour_2d = contour.reshape(-1, 2)
        simplified_contour = rdp(contour_2d, epsilon=simplification_tolerance)

        if len(simplified_contour) < 2:
            continue

        segments = []
        for i in range(len(simplified_contour) - 1):
            start_x_px, start_y_px = simplified_contour[i]
            end_x_px, end_y_px = simplified_contour[i + 1]

            # Scale
            scaled_start_x = start_x_px * final_pixel_to_meter_scale
            scaled_start_y = start_y_px * final_pixel_to_meter_scale
            scaled_end_x = end_x_px * final_pixel_to_meter_scale
            scaled_end_y = end_y_px * final_pixel_to_meter_scale

            # Translate and flip Y
            offset_x = image_center_x_px * final_pixel_to_meter_scale
            offset_y = image_center_y_px * final_pixel_to_meter_scale

            start_complex = complex(scaled_start_x - offset_x, -(scaled_start_y - offset_y))
            end_complex = complex(scaled_end_x - offset_x, -(scaled_end_y - offset_y))

            segments.append(Line(start_complex, end_complex))

        if segments:
            svg_paths.append(Path(*segments))

    # Save SVG without viewBox (not supported by svgpathtools)
    wsvg(
        svg_paths,
        filename=output_file,
        attributes=[{'stroke': 'black', 'stroke-width': '0.002', 'fill': 'none'} for _ in svg_paths]
    )

    drawing_width = width * final_pixel_to_meter_scale
    drawing_height = height * final_pixel_to_meter_scale

    print(f"SVG saved to {output_file} with {len(svg_paths)} paths. Drawing size: {drawing_width:.3f}m x {drawing_height:.3f}m")
    return output_file
