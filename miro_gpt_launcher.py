#!/usr/bin/env python3

import os
import sys
import time
import json
import rospy
import logging
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Logging configuration to override previous settings
logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] %(message)s', force=True)

# Ensure face_id and other modules can be found when running with rosrun
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from face_id.authentication import FaceAuthenticator
# TODO: Uncomment and implement when ready
# from face_id.registration import register_new_face
from config import (
    PROFILES_DIR,
    DEBUG_FACE_RECOGNITION,
    SEARCH_TIMEOUT,
    SEARCH_INTERVAL,
    CAMERA_SOURCE
)
from debug_utils import (
    log_info,
    log_warning,
    log_error,
    log_face_recognition
)

# ROS publishers
head_pub = None
voice_pub = None
motor_pub = None

# Global state
found_user = False
user_name = None
search_start_time = None
auth = None  # Global reference to FaceAuthenticator

def speak(text):
    """Publish text to MiRo's voice system."""
    if voice_pub:
        voice_pub.publish(text)
        log_info(f"[SPEAK] {text}")
    else:
        log_warning("Voice publisher not initialized.")

def turn_body():
    """Simulate body turning to scan for a face."""
    if motor_pub:
        twist = Twist()
        twist.angular.z = 0.5
        motor_pub.publish(twist)
        logging.debug("🌀 MiRo rotating body to search.")

def stop_body():
    """Stop MiRo's rotation."""
    if motor_pub:
        twist = Twist()
        twist.angular.z = 0.0
        motor_pub.publish(twist)
        logging.debug("⛔ MiRo stopped rotating.")

def load_profile(name):
    """Load a user's profile from disk."""
    try:
        profile_path = os.path.join(PROFILES_DIR, f"{name}.json")
        with open(profile_path, "r") as f:
            profile = json.load(f)
        rospy.set_param("miro_active_profile", name)
        log_info(f"[PROFILE] Loaded profile for {name}")
        return profile
    except FileNotFoundError:
        log_warning(f"[PROFILE] Not found for: {name}")
        return None
    except Exception as e:
        log_error(f"[PROFILE] Failed to load {name}: {e}")
        return None

def search_loop():
    """Search for a face and optionally trigger registration."""
    global found_user, user_name, auth
    sweep_count = 0
    speak("Hmm... Hello? Anyone there?")

    while not found_user and (time.time() - search_start_time < SEARCH_TIMEOUT):
        logging.debug("📷 Capturing frame for recognition...")

        user_name = auth.recognize_face()

        if user_name and user_name != "Unknown":
            log_face_recognition(f"✅ Face recognized as: {user_name}")
            found_user = True
            stop_body()
            speak(f"Oh! Hello {user_name}. It's good to see you again.")
            load_profile(user_name)
            return
        else:
            log_face_recognition("❌ No known face recognized.")

        turn_body()
        time.sleep(SEARCH_INTERVAL)
        sweep_count += 1

    # Timeout: no known face found
    stop_body()
    speak("Hello there! I don’t believe we’ve met.")
    log_info("[SEARCH] Timeout — no face recognized.")

    speak("Would you like me to remember you? Please look directly at me.")
    # TODO: Implement registration logic here
    # success, new_name = register_new_face()
    # if success:
    #     speak(f"Nice to meet you, {new_name}! I’ll remember your face.")
    #     rospy.set_param("miro_active_profile", new_name)
    #     found_user = True

def main():
    """Entry point for startup search node."""
    global head_pub, voice_pub, motor_pub, search_start_time, auth

    logging.info("🚀 startup_search_node.py started")
    rospy.init_node("startup_search_node")

    logging.debug("📡 Initialising ROS publishers...")
    head_pub = rospy.Publisher("/miro/command/head", Twist, queue_size=1)
    voice_pub = rospy.Publisher("/miro/command/voice", String, queue_size=1)
    motor_pub = rospy.Publisher("/miro/command/velocity", Twist, queue_size=1)  # Updated to match config

    time.sleep(2)  # Let publishers fully initialise

    # 🔄 Initialise FaceAuthenticator early so ROS camera subscriber becomes active immediately
    log_info("🔍 Initialising FaceAuthenticator...")
    auth = FaceAuthenticator()

    search_start_time = time.time()
    logging.debug("🔍 Entering face search loop...")
    search_loop()

    rospy.set_param("miro_ready", found_user)
    rospy.spin()

if __name__ == "__main__":
    main()
