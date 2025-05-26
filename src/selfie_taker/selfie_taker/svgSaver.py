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
<<<<<<< HEAD
    contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contour_time = time.time()
=======
    contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
>>>>>>> 6de09c8 (ok re run)

    print("Processing contours and generating SVG paths...")

    base_pixel_to_meter_scale = 0.001  # 1 pixel = 1mm
    final_pixel_to_meter_scale = base_pixel_to_meter_scale * svg_output_scale_factor

    image_center_x_px = width / 2.0
    image_center_y_px = height / 2.0

<<<<<<< HEAD
    MIN_CONTOUR_LENGTH = 5

    # Step 3: Simplify contours and convert to line segments
    for contour in tqdm(contours, desc="Simplifying and converting contours", unit="contour"):
        if len(contour) < MIN_CONTOUR_LENGTH:
=======
    for contour in tqdm(contours, desc="SVG Generation"):
        if len(contour) < 2:
>>>>>>> 6de09c8 (ok re run)
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

<<<<<<< HEAD
    line_collection_time = time.time()

    print(f"Generating SVG with {len(svg_lines)} line segments...")

    # Step 4: Write SVG with progress bar
    drawing_width_mm = width * final_pixel_to_meter_scale * 1000
    drawing_height_mm = height * final_pixel_to_meter_scale * 1000

    dwg = svgwrite.Drawing(output_file, size=(f"{drawing_width_mm}mm", f"{drawing_height_mm}mm"))
    for start, end in tqdm(svg_lines, desc="Writing SVG lines", unit="line"):
        dwg.add(dwg.line(start=start, end=end, stroke='black', stroke_width=0.002))

    dwg.save()
    save_time = time.time()

    # Step 5: Print timing summary
    print(f"\nSVG saved to {output_file} with {len(svg_lines)} lines.")
    print(f"Drawing size: {width * final_pixel_to_meter_scale:.3f}m x {height * final_pixel_to_meter_scale:.3f}m")
    print(f"\nTiming breakdown:")
    print(f"- Contour extraction: {(contour_time - start_time):.2f}s")
    print(f"- Line simplification & collection: {(line_collection_time - contour_time):.2f}s")
    print(f"- SVG writing: {(save_time - line_collection_time):.2f}s")
    print(f"- Total time: {(save_time - start_time):.2f}s")
=======
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
>>>>>>> 6de09c8 (ok re run)

    print(f"SVG saved to {output_file} with {len(svg_paths)} paths. Drawing size: {drawing_width:.3f}m x {drawing_height:.3f}m")
    return output_file
