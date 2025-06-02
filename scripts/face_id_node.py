#!/usr/bin/env python3

import os
import cv2
print("OpenCV version:", cv2.__version__)
import dlib
import rospy
import logging
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from com3528_individual.msg import FaceData

import sys
import os

# Add project base directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
MODEL_DIR = os.path.join(os.path.dirname(__file__), "..", "models")

from settings.config import (
    FACES_DIR,
    FACE_MATCH_THRESHOLD,
    CAMERA_SOURCE_RIGHT,
    DEBUG_FACE_RECOGNITION,
    TOPIC_FACE_ID
)
from settings.debug_utils import log_info, log_error, log_warning, log_face_recognition

# === Logging Setup ===
logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] %(message)s', force=True)

class FaceRecognizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.face_detector = dlib.get_frontal_face_detector()
        self.shape_predictor = dlib.shape_predictor(os.path.expanduser("~/models/shape_predictor_68_face_landmarks.dat"))
        self.face_rec_model = dlib.face_recognition_model_v1(
            os.path.join(MODEL_DIR, "dlib_face_recognition_resnet_model_v1.dat")
        )
        self.known_faces = []
        self.known_names = []

        self.publisher = rospy.Publisher(TOPIC_FACE_ID, FaceData, queue_size=10)

        self.load_known_faces(FACES_DIR)
        rospy.Subscriber(CAMERA_SOURCE_RIGHT, CompressedImage, self.image_callback)
        log_info("[INIT] FaceRecognizer node initialised.")

    def load_known_faces(self, folder):
        for filename in os.listdir(folder):
            if filename.endswith(('.jpg', '.jpeg', '.png')):
                name = os.path.splitext(filename)[0]
                path = os.path.join(folder, filename)
                try:
                    img = cv2.imread(path)
                    if img is None:
                        raise ValueError("Image read as None")
                    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    dets = self.face_detector(rgb)
                    if not dets:
                        raise ValueError("No face detected")
                    shape = self.shape_predictor(rgb, dets[0])
                    face_desc = np.array(self.face_rec_model.compute_face_descriptor(rgb, shape))
                    self.known_faces.append(face_desc)
                    self.known_names.append(name)
                    log_info(f"[LOAD] Loaded face encoding for: {name}")
                except Exception as e:
                    log_warning(f"[SKIP] Failed to load {filename}: {e}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            dets = self.face_detector(rgb)

            for det in dets:
                shape = self.shape_predictor(rgb, det)
                desc = np.array(self.face_rec_model.compute_face_descriptor(rgb, shape))
                distances = np.linalg.norm(np.array(self.known_faces) - desc, axis=1)

                if len(distances) > 0:
                    min_idx = np.argmin(distances)
                    min_dist = distances[min_idx]
                    name = self.known_names[min_idx] if min_dist < FACE_MATCH_THRESHOLD else "Unknown"
                else:
                    name, min_dist = "Unknown", 1.0

                x, y, w, h = det.left(), det.top(), det.width(), det.height()
                is_centered = self.is_face_centered(frame.shape, x, y, w, h)

                msg_out = FaceData(
                    name=name,
                    confidence=round(1 - min_dist, 3),
                    x=x, y=y, width=w, height=h,
                    is_centered=is_centered
                )
                self.publisher.publish(msg_out)

                if name != "Unknown":
                    log_face_recognition(f"Recognized: {name} @ {round(1 - min_dist, 3)}")
                    rospy.set_param("/miro_active_user", name)
                else:
                    log_face_recognition("Face not recognized")

        except Exception as e:
            log_error(f"[ERROR] Failed in image_callback: {e}")

    def is_face_centered(self, shape, x, y, w, h):
        cx, cy = x + w / 2, y + h / 2
        img_h, img_w = shape[:2]
        center_x, center_y = img_w / 2, img_h / 2
        tol_x, tol_y = img_w * 0.15, img_h * 0.15
        return abs(cx - center_x) < tol_x and abs(cy - center_y) < tol_y

if __name__ == '__main__':
    rospy.init_node('face_id_node', anonymous=False)
    recognizer = FaceRecognizer()
    rospy.spin()
