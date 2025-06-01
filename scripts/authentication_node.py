#!/usr/bin/env python3
"""
authentication_node.py
Handles real-time face recognition and optional user registration for MiRoGPT.
"""

import rospy
from std_msgs.msg import String
from face_id.simple_facerec import SimpleFacerec
from face_id.registration import register_new_face, sanitize_username
from debug_utils import log_info, log_warning, log_error, log_face_recognition
from config import PROFILE_DIR, CAMERA_SOURCE, DEBUG_FACE_RECOGNITION
import os

# ROS publisher to send recognized name
pub = None

# Initialize face recognizer
sfr = SimpleFacerec()
encoding_path = os.path.join(os.path.dirname(__file__), "face_id/encodings")
sfr.load_encoding_images(encoding_path)

def callback(data):
    """Callback function for incoming face recognition trigger messages."""
    try:
        name = sfr.recognize_from_camera()

        if name:
            log_face_recognition(f"Recognized user: {name}")
            pub.publish(name)
        else:
            log_face_recognition("Unknown face detected.")
            pub.publish("Unknown")

            # Ask user for registration
            response = input("[PROMPT] Would you like MiRo to remember your face? (y/n): ").strip().lower()
            if response == "y":
                name_input = input("Please enter your name: ")
                username = sanitize_username(name_input)

                success = register_new_face(username)
                if success:
                    log_info(f"Face registered for new user: {username}")
                    print("[INFO] Restart the node to update face encodings.")
                else:
                    log_warning("Face registration failed.")

    except Exception as e:
        log_error(f"[AuthNode] Error during face recognition: {str(e)}")

def main():
    global pub
    rospy.init_node("authentication_node")
    pub = rospy.Publisher("miro/user_name", String, queue_size=1)
    rospy.Subscriber("miro/face_trigger", String, callback)
    log_info("[AuthNode] Authentication node started.")
    rospy.spin()

if __name__ == "__main__":
    main()
