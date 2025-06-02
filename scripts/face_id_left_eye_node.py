#!/usr/bin/env python3

import os
import cv2
import dlib
import rospy
import numpy as np
import traceback
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from com3528_individual.msg import FaceData

from settings.config import (
    FACES_DIR,
    FACE_MATCH_THRESHOLD,
    CAMERA_SOURCE_LEFT,
    DEBUG_FACE_RECOGNITION,
    TOPIC_FACE_ID_LEFT,
    RECOGNITION_FOLDERS,
    SHAPE_PREDICTOR_PATH,
    FACE_RECOGNITION_MODEL_PATH
)
from settings.debug_utils import log_info, log_error, log_warning, log_face_recognition

class FaceRecognizerNode:
    def __init__(self):
        rospy.init_node('face_id_LEFT_node')
        self.bridge = CvBridge()

        # Load dlib models
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(SHAPE_PREDICTOR_PATH)
        self.encoder = dlib.face_recognition_model_v1(FACE_RECOGNITION_MODEL_PATH)

        # Publisher for outgoing data
        self.publisher = rospy.Publisher(TOPIC_FACE_ID_LEFT, FaceData, queue_size=10)

        # Load known faces from configured folders
        self.known_faces, self.known_names, self.known_folders = self.load_all_known_faces()
        if not self.known_faces:
            log_warning("No known faces were loaded. Recognition will not work.")

        # Subscribe to camera stream
        rospy.Subscriber(CAMERA_SOURCE_LEFT, CompressedImage, self.callback)
        log_info("[BOOT - LEFT] Node initialized and camera subscribed.")

        self.last_user_name = None

    def load_all_known_faces(self):
        encodings, names, folders = [], [], []
        for folder in RECOGNITION_FOLDERS:
            full_path = os.path.join(FACES_DIR, folder)
            if not os.path.exists(full_path):
                log_warning(f"[LOAD FACES - LEFT] Folder not found: {full_path}")
                continue
            for file in os.listdir(full_path):
                if file.lower().endswith((".jpg", ".png", ".jpeg")):
                    img_path = os.path.join(full_path, file)
                    name = os.path.splitext(file)[0]
                    img = cv2.imread(img_path)
                    if img is None:
                        log_warning(f"[LOAD FACES - LEFT] Failed to load: {img_path}")
                        continue
                    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    dets = self.detector(rgb)
                    if not dets:
                        log_warning(f"[LOAD FACES - LEFT] No face in image: {img_path}")
                        continue
                    shape = self.predictor(rgb, dets[0])
                    desc = np.array(self.encoder.compute_face_descriptor(rgb, shape))
                    encodings.append(desc)
                    names.append(name)
                    folders.append(folder)
        return encodings, names, folders

    def callback(self, msg):
        try:
            try:
                frame = self.bridge.compressed_imgmsg_to_cv2(msg)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                log_info("[CALLBACK - FRAME OK] Frame successfully decoded.")
            except Exception as e:
                log_error(f"[CALLBACK - DECODE ERROR] Failed to convert image: {e}")
                import traceback
                traceback.print_exc()
                return
            dets = self.detector(rgb)

            if not dets:
                log_info("[CALLBACK - LEFT] No faces detected in frame.")
                return

            for det in dets:
                shape = self.predictor(rgb, det)
                desc = np.array(self.encoder.compute_face_descriptor(rgb, shape))

                distances = np.linalg.norm(np.array(self.known_faces) - desc, axis=1)

                if len(distances) > 0:
                    min_idx = np.argmin(distances)
                    min_dist = distances[min_idx]
                    if min_dist < FACE_MATCH_THRESHOLD:
                        name = self.known_names[min_idx]
                        folder = self.known_folders[min_idx]
                    else:
                        name, folder = "Unknown", "Unknown"
                        min_dist = 1.0
                else:
                    name, folder = "Unknown", "Unknown"
                    min_dist = 1.0

                x, y, w, h = det.left(), det.top(), det.width(), det.height()
                cx, cy = x + w / 2, y + h / 2
                img_h, img_w = frame.shape[:2]
                tol_x, tol_y = img_w * 0.15, img_h * 0.15
                is_centered = abs(cx - img_w / 2) < tol_x and abs(cy - img_h / 2) < tol_y

                msg_out = FaceData(
                    name=name,
                    confidence=round(1 - min_dist, 3),
                    x=x,
                    y=y,
                    width=w,
                    height=h,
                    is_centered=is_centered
                )

                self.publisher.publish(msg_out)
                log_info(f"[PUBLISH - LEFT] Sent FaceData for: {msg_out.name} (Conf: {msg_out.confidence})")

                if name != self.last_user_name:
                    rospy.set_param("/miro_active_user", name)
                    rospy.set_param("/miro_user_folder", folder)
                    self.last_user_name = name

                log_face_recognition(f"[LEFT] {name} ({folder}) - Conf: {round(1 - min_dist, 3)}")

        except Exception as e:
            log_error(f"[CALLBACK ERROR - LEFT] {e}")
            traceback.print_exc()

if __name__ == '__main__':
    try:
        log_info("[BOOT - LEFT] Starting face recognizer node...")
        FaceRecognizerNode()
        rospy.spin()
    except Exception as e:
        log_error(f"[FATAL INIT ERROR - LEFT] {e}")
        traceback.print_exc()
