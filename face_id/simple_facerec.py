# simple_facerec.py

import cv2
import face_recognition
import os
import numpy as np
import logging
from config import (
    KNOWN_FACES_DIR,         # Directory containing known user face images
    TOLERANCE,               # Threshold for face match confidence
    MODEL,                   # Face detection model (e.g., 'hog' or 'cnn')
    DEBUG_FACE_RECOGNITION   # Debugging flag for face recognition logging
)

# Set up logging configuration
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

class SimpleFaceRec:
    """
    A class to manage face recognition.
    Loads known user face encodings from a structured directory and matches against incoming video frames.
    """
    def __init__(self):
        self.known_face_encodings = []  # List of face encodings
        self.known_face_names = []      # Corresponding list of user names
        self._load_known_faces()        # Populate face database at init

    def _load_known_faces(self):
        """
        Load and encode user face images stored in subfolders under KNOWN_FACES_DIR.
        Each subfolder should be named after the person and contain one or more images.
        """
        if DEBUG_FACE_RECOGNITION:
            logging.debug(f"Loading known faces from {KNOWN_FACES_DIR}...")

        for name in os.listdir(KNOWN_FACES_DIR):
            person_dir = os.path.join(KNOWN_FACES_DIR, name)
            if not os.path.isdir(person_dir):
                continue

            for filename in os.listdir(person_dir):
                filepath = os.path.join(person_dir, filename)
                try:
                    image = face_recognition.load_image_file(filepath)
                    encoding = face_recognition.face_encodings(image)
                    if encoding:
                        # Store the encoding and associate with name
                        self.known_face_encodings.append(encoding[0])
                        self.known_face_names.append(name)
                        if DEBUG_FACE_RECOGNITION:
                            logging.debug(f"Encoded {filename} for {name}.")
                    else:
                        logging.warning(f"No face found in image: {filename}")
                except Exception as e:
                    logging.error(f"Failed to load {filename}: {e}")

    def detect_known_face(self, frame):
        """
        Compares faces found in the input frame to known encodings.
        Returns the name of the best match or 'Unknown' if no match is found.
        """
        rgb_frame = frame[:, :, ::-1]  # Convert from BGR to RGB format
        face_locations = face_recognition.face_locations(rgb_frame, model=MODEL)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        for encoding, location in zip(face_encodings, face_locations):
            # Compare detected face encoding with known ones
            results = face_recognition.compare_faces(self.known_face_encodings, encoding, TOLERANCE)
            face_distances = face_recognition.face_distance(self.known_face_encodings, encoding)
            best_match_index = np.argmin(face_distances) if face_distances.size > 0 else -1

            if best_match_index != -1 and results[best_match_index]:
                matched_name = self.known_face_names[best_match_index]
                logging.info(f"Recognised face: {matched_name}")
                return matched_name

        logging.info("No known face detected.")
        return "Unknown"

if __name__ == "__main__":
    # Manual testing: capture frames and display recognition results
    sfr = SimpleFaceRec()
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            logging.error("Failed to grab frame from camera.")
            break

        name = sfr.detect_known_face(frame)
        # Overlay recognised name on frame for visual feedback
        cv2.putText(frame, name, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Face Recognition", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
