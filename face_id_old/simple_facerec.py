# simple_facerec.py

import cv2
import face_recognition
import os
import numpy as np
import logging
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from config import (
    FACES_DIR,
    FACE_MATCH_THRESHOLD,
    CAMERA_SOURCE,
    DEBUG_FACE_RECOGNITION
)

# Constants
MODEL = "hog"  # Use 'cnn' if GPU acceleration is available

# Set up logging configuration
logging.basicConfig(level=logging.DEBUG if DEBUG_FACE_RECOGNITION else logging.INFO,
                    format='[%(levelname)s] %(message)s')

class MiRoCameraFeed:
    def __init__(self, topic=CAMERA_SOURCE):
        self.bridge = CvBridge()
        self.latest_frame = None
        self.received_frame = False
        self.sub = rospy.Subscriber(topic, CompressedImage, self.callback, queue_size=1)
        logging.info(f"📡 Subscribed to MiRo camera topic: {topic}")

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.received_frame = True
            logging.debug("🟢 Frame received and decoded from MiRo camera.")
        except Exception as e:
            logging.error(f"❌ Failed to decode camera image: {e}")

    def get_frame(self):
        if not self.received_frame:
            logging.debug("⚠️ No frame received yet from MiRo camera.")
        return self.latest_frame

class SimpleFaceRec:
    """
    A class to manage face recognition.
    Loads known user face encodings and matches against incoming frames.
    """
    def __init__(self):
        self.known_face_encodings = []
        self.known_face_names = []
        self._load_known_faces()

    def _load_known_faces(self):
        """
        Load face images stored in subdirectories under FACES_DIR.
        Each subfolder should be named after the person.
        """
        if DEBUG_FACE_RECOGNITION:
            logging.debug(f"📁 Loading known faces from {FACES_DIR}...")

        for name in os.listdir(FACES_DIR):
            person_dir = os.path.join(FACES_DIR, name)
            if not os.path.isdir(person_dir):
                continue

            for filename in os.listdir(person_dir):
                filepath = os.path.join(person_dir, filename)
                try:
                    image = face_recognition.load_image_file(filepath)
                    encoding = face_recognition.face_encodings(image)
                    if encoding:
                        self.known_face_encodings.append(encoding[0])
                        self.known_face_names.append(name)
                        if DEBUG_FACE_RECOGNITION:
                            logging.debug(f"✅ Encoded {filename} for {name}.")
                    else:
                        logging.warning(f"⚠️ No face found in image: {filename}")
                except Exception as e:
                    logging.error(f"❌ Failed to load {filename}: {e}")

    def detect_known_face(self, frame):
        """
        Compares detected faces in the frame to known encodings.
        Returns best match or 'Unknown'.
        """
        if frame is None:
            logging.warning("❌ No frame to process in detect_known_face.")
            return "Unknown"

        rgb_frame = frame[:, :, ::-1]
        face_locations = face_recognition.face_locations(rgb_frame, model=MODEL)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        if DEBUG_FACE_RECOGNITION:
            logging.debug(f"🔍 Detected {len(face_encodings)} face(s) in frame.")

        for encoding, location in zip(face_encodings, face_locations):
            results = face_recognition.compare_faces(self.known_face_encodings, encoding, FACE_MATCH_THRESHOLD)
            face_distances = face_recognition.face_distance(self.known_face_encodings, encoding)
            best_match_index = np.argmin(face_distances) if face_distances.size > 0 else -1

            if best_match_index != -1 and results[best_match_index]:
                matched_name = self.known_face_names[best_match_index]
                logging.info(f"🟢 Recognised face: {matched_name}")
                return matched_name

        logging.info("❓ No known face detected.")
        return "Unknown"

if __name__ == "__main__":
    rospy.init_node("simple_face_rec_node")
    sfr = SimpleFaceRec()
    camera = MiRoCameraFeed(CAMERA_SOURCE)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        frame = camera.get_frame()
        if frame is None:
            logging.debug("⌛ Waiting for camera frame...")
            rate.sleep()
            continue

        name = sfr.detect_known_face(frame)
        cv2.putText(frame, name, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Face Recognition", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cv2.destroyAllWindows()
