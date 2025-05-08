import cv2
import numpy as np
import rembg
import time

def detect_and_crop_face():
    # Capture Image from Webcam
    cap = cv2.VideoCapture(0)  # Ensure Bluetooth is off to avoid conflicts
    time.sleep(2)  # Necessary delay for the camera to initialize

    ret, image = cap.read()
    cap.release()

    if not ret:
        print("Error: Could not read image from camera.")
        return None

    # Remove Background Using rembg
    image_nobg = rembg.remove(image)
    cv2.imshow("Image without Background", image_nobg)
    if image_nobg is None:
        print("Error: Image not found. Check the file path!")
        return None

    # Convert RGBA to BGR
    image_nobg = cv2.cvtColor(image_nobg, cv2.COLOR_BGRA2BGR)

    # Load Haar Cascade for Face Detection
    face_classifier = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )

    # Detect Faces
    faces = face_classifier.detectMultiScale(
        image_nobg, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
    )

    if len(faces) == 0:
        print("No face detected.")
        return None
    

    largest_face_Area = 0
    largest_face = None
    area = 0

    for (x_face, y_face, w_face, h_face) in faces:
        area = w_face * h_face
        if area > largest_face_Area:
            largest_face_Area = area
            largest_face = (x_face, y_face, w_face, h_face)
            

    xface = largest_face[0]
    yface = largest_face[1]
    wface = largest_face[2]
    hface = largest_face[3]
        # Expand the cropping space by 10% in each direction
    x_start = max(0, int(x_face - 0.1 * w_face))
    y_start = max(0, int(y_face - 0.1 * h_face))
    x_end = min(image_nobg.shape[1], int(x_face + w_face + 0.1 * w_face))
    y_end = min(image_nobg.shape[0], int(y_face + h_face + 0.1 * h_face))

        # Crop the expanded face region
    face_region = image_nobg[y_start:y_end, x_start:x_end]

    return face_region  # Return the cropped face region if needed