# authentication.py

import rospy
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from .simple_facerec import SimpleFaceRec
import logging
from config import (
    CAMERA_SOURCE,
    DEBUG_FACE_RECOGNITION
)

# Configure logging
logging.basicConfig(
    level=logging.DEBUG if DEBUG_FACE_RECOGNITION else logging.INFO,
    format='[%(levelname)s] %(message)s',
    force=True
)

class FaceAuthenticator:
    """
    FaceAuthenticator wraps around SimpleFaceRec to subscribe to MiRo's ROS camera stream
    and perform face recognition on incoming compressed image messages.
    """
    def __init__(self):
        logging.info("🔍 Initialising FaceAuthenticator (ROS-based)...")
        self.sfr = SimpleFaceRec()
        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()

        # Subscribe to ROS camera topic
        rospy.Subscriber(CAMERA_SOURCE, CompressedImage, self._image_callback, queue_size=1)
        logging.info(f"📡 Subscribed to ROS camera topic: {CAMERA_SOURCE}")

    def _image_callback(self, msg):
        """
        Internal callback for camera image messages.
        Converts compressed ROS image to OpenCV frame.
        """
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = frame
            logging.debug("🖼️ New frame received from ROS camera stream.")
        except Exception as e:
            logging.error(f"❌ Failed to decode image frame: {e}")

    def recognize_face(self):
        """
        Attempt to identify a known face from the latest frame received from MiRo.
        :return: Name of recognised user or None if no known face is found.
        """
        with self.lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None

        if frame is None:
            logging.warning("⚠️ No frame available yet from ROS image stream.")
            return None

        logging.debug("🔎 Processing frame for face recognition...")
        name = self.sfr.detect_known_face(frame)

        if name != "Unknown":
            logging.info(f"✅ User recognised: {name}")
            return name
        else:
            logging.debug("❓ Face detected but not recognised.")
            return None

    def release(self):
        """
        No-op in ROS context, provided for compatibility.
        """
        pass

if __name__ == "__main__":
    rospy.init_node("face_auth_debug")
    auth = FaceAuthenticator()

    try:
        logging.info("🧪 Face recognition test started. Ctrl+C to exit.")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            user = auth.recognize_face()
            if user:
                print(f"User identified: {user}")
            rate.sleep()
    except rospy.ROSInterruptException:
        logging.info("🔻 ROS node interrupted. Exiting cleanly.")
