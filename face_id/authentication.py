# authentication.py

import cv2
from simple_facerec import SimpleFaceRec
import logging
from config import (
    CAMERA_SOURCE,
    DEBUG_FACE_RECOGNITION
)

# Configure logging
logging.basicConfig(level=logging.DEBUG if DEBUG_FACE_RECOGNITION else logging.INFO,
                    format='[%(levelname)s] %(message)s')

class FaceAuthenticator:
    """
    FaceAuthenticator wraps around SimpleFaceRec to manage the camera stream and perform
    face recognition from real-time input.
    """
    def __init__(self):
        """
        Initialise the face recognition system and connect to the camera.
        """
        self.sfr = SimpleFaceRec()  # Initialise face recogniser
        self.camera = cv2.VideoCapture(CAMERA_SOURCE)  # Open webcam or MiRo camera
        if not self.camera.isOpened():
            logging.error("Camera could not be accessed.")

    def recognize_face(self):
        """
        Capture a frame and attempt to identify a known face.
        :return: Name of recognised user or None if no known face is found.
        """
        ret, frame = self.camera.read()
        if not ret:
            logging.warning("Failed to read frame from camera.")
            return None

        name = self.sfr.detect_known_face(frame)
        if name != "Unknown":
            logging.info(f"User recognised: {name}")
            return name
        else:
            logging.debug("Face detected but not recognised.")
        return None

    def release(self):
        """
        Release the webcam resource.
        """
        self.camera.release()

if __name__ == "__main__":
    # Debug mode: test the face authentication system in isolation
    auth = FaceAuthenticator()
    try:
        logging.info("Starting face recognition test. Press Ctrl+C to exit.")
        while True:
            user = auth.recognize_face()
            if user:
                print(f"User identified: {user}")
    except KeyboardInterrupt:
        logging.info("Shutting down test loop.")
    finally:
        auth.release()
