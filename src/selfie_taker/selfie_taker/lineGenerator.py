import cv2
import numpy as np

def lineGenerator(image):
    # Convert to Grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Normalize for Better Edge Detection
    gray_norm = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    cv2.imshow("Normalized Gray Image", gray_norm)

    # Apply CLAHE to Enhance Face Details
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_clahe = clahe.apply(gray_norm)

    # Estimate the Hair Region (Top 20% of the Image)
    hair_region_height = int(0.3 * gray_clahe.shape[0])  # Top 20% of the image height
    hair_region = gray_clahe[:hair_region_height, :]  # Crop the top 20% of the image

    # Apply Stronger Smoothing to the Hair Region
    hair_region_blurred = cv2.GaussianBlur(hair_region, (7, 7), 0)  # Stronger blur for fewer edges

    # Replace the Smoothed Hair Region in the Original Image
    gray_clahe[:hair_region_height, :] = hair_region_blurred

    # Apply Gaussian Blur to the Entire Image (Optional)
    blurred = cv2.GaussianBlur(gray_clahe, (3, 3), 0)
    # cv2.imshow("Blurred Image", blurred)

    # Canny Edge Detection
    edges = cv2.Canny(blurred, 100, 150)
    cv2.imshow("Canny Edges", edges)
    

    # Display the edges for debugging
    cv2.imshow('Edges with Blurred Hair Region', edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    return edges