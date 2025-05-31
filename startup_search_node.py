import time
import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from authentication import FaceAuthenticator
from registration import register_new_face
from config import (
    PROFILE_DIR,
    DEBUG_FACE_RECOGNITION,
    SEARCH_TIMEOUT,
    SEARCH_INTERVAL
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

def stop_body():
    """Stop MiRo's rotation."""
    if motor_pub:
        twist = Twist()
        twist.angular.z = 0.0
        motor_pub.publish(twist)

def load_profile(name):
    """Load a user's profile from disk."""
    try:
        with open(f"{PROFILE_DIR}{name}.json", "r") as f:
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
    global found_user, user_name
    sweep_count = 0
    speak("Hmm... Hello? Anyone there?")
    auth = FaceAuthenticator()

    while not found_user and (time.time() - search_start_time < SEARCH_TIMEOUT):
        user_name = auth.recognize_face()
        log_face_recognition(f"Face recognized as: {user_name}" if user_name else "Face not recognized")

        if user_name:
            found_user = True
            stop_body()
            speak(f"Oh! Hello {user_name}. It's good to see you again.")
            load_profile(user_name)
            return

        turn_body()
        time.sleep(SEARCH_INTERVAL)
        sweep_count += 1

    # Timeout: no known face found
    stop_body()
    speak("Hello there! I don’t believe we’ve met.")
    log_info("[SEARCH] Timeout — no face recognized.")

    # Register new user
    speak("Would you like me to remember you? Please look directly at me.")
    success, new_name = register_new_face()
    if success:
        speak(f"Nice to meet you, {new_name}! I’ll remember your face.")
        rospy.set_param("miro_active_profile", new_name)
        found_user = True

def main():
    """Entry point for startup search node."""
    global head_pub, voice_pub, motor_pub, search_start_time

    rospy.init_node("startup_search_node")
    head_pub = rospy.Publisher("/miro/command/head", Twist, queue_size=1)
    voice_pub = rospy.Publisher("/miro/command/voice", String, queue_size=1)
    motor_pub = rospy.Publisher("/miro/command/motor", Twist, queue_size=1)

    time.sleep(2)  # Let publishers fully initialise
    search_start_time = time.time()
    search_loop()

    rospy.set_param("miro_ready", found_user)
    rospy.spin()

if __name__ == "__main__":
    main()
