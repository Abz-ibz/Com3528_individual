#!/usr/bin/env python3

import os
import cv2
import rospy
import logging
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from com3528_individual.msg import FaceData

import sys

# Add project base directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from settings.config import (
    CENTERING_TOLERANCE,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    CAMERA_SOURCE_LEFT,
    DEBUG_FACE_TRACKING,
    TOPIC_FACE_TRACK
)
from settings.debug_utils import log_info, log_warning, log_error, log_face_tracking

# === Logging Setup ===
logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] %(message)s', force=True)

class FaceTrackerNode:
    def __init__(self):
        rospy.init_node("face_track_node", anonymous=False)

        # OpenCV bridge for ROS image messages
        self.bridge = CvBridge()

        # Load Haar cascade for face detection
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

        # Subscriber for left eye camera
        self.image_sub = rospy.Subscriber(CAMERA_SOURCE_LEFT, CompressedImage, self.image_callback)

        # Publisher for face tracking data
        self.face_pub = rospy.Publisher(TOPIC_FACE_TRACK, FaceData, queue_size=1)

        log_info("[TRACKER] Haar cascade face detection node initialized.")

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV format
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            log_error(f"[TRACKER] CvBridge error: {e}")
            return

        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        if DEBUG_FACE_TRACKING:
            log_face_tracking(f"[DEBUG] Detected {len(faces)} faces in frame")

        for (x, y, w, h) in faces:
            # Calculate center of face and check if it is centered in the frame
            cx = x + w // 2
            centered = abs(cx - FRAME_WIDTH // 2) <= CENTERING_TOLERANCE * FRAME_WIDTH

            # Create and publish FaceData message
            self.face_pub.publish(FaceData(
                name="detected",
                confidence=0.9,
                x=x,
                y=y,
                width=w,
                height=h,
                is_centered=centered
            ))

            if DEBUG_FACE_TRACKING:
                log_face_tracking(f"[DEBUG] Face bbox: x={x}, y={y}, w={w}, h={h}, centered={centered}")
                # Draw bounding box and centering line
                color = (0, 255, 0) if centered else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.line(frame, (FRAME_WIDTH // 2, 0), (FRAME_WIDTH // 2, FRAME_HEIGHT), (255, 255, 0), 1)
                cv2.imshow("Tracker-Free Face View", frame)
                cv2.waitKey(1)

            break  # Only process first face for simplicity

if __name__ == '__main__':
    try:
        FaceTrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        log_info("[TRACKER] Shutdown requested")
    finally:
        cv2.destroyAllWindows()
