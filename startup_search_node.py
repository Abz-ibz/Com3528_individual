# startup_search_node.py
# Controls MiRo's startup face search behavior, recognition, and profile activation

import time
import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from authentication import FaceAuthenticator
from config import PROFILE_DIR, DEBUG_FACE_RECOGNITION, SEARCH_TIMEOUT, SEARCH_INTERVAL

# ROS publishers
head_pub = None
voice_pub = None
motor_pub = None

# Global state
found_user = False
user_name = None
search_start_time = None

def speak(text):
    """Publish text to the MiRo voice system."""
    voice_pub.publish(text)

def turn_head(yaw):
    """Simulate MiRo's head yaw rotation."""
    twist = Twist()
    twist.angular.z = yaw
    head_pub.publish(twist)

def turn_body():
    """Simulate MiRo's slow clockwise body turn."""
    twist = Twist()
    twist.angular.z = 0.5
    motor_pub.publish(twist)

def stop_body():
    """Stop MiRo's rotational movement."""
    twist = Twist()
    twist.angular.z = 0.0
    motor_pub.publish(twist)

def load_profile(name):
    """Attempt to load user profile JSON and set ROS param for profile activation."""
    try:
        with open(f"{PROFILE_DIR}{name}.json", "r") as f:
            profile = json.load(f)
        rospy.set_param("miro_active_profile", name)
        rospy.loginfo(f"[PROFILE] Loaded profile for {name}")
        return profile
    except FileNotFoundError:
        rospy.logwarn("[PROFILE] Profile not found for user: " + name)
        return None

def search_loop():
    """Main loop to search for a known face. If found, activate user profile."""
    global found_user, user_name
    sweep_count = 0
    speak("Hmm... Hello? Anyone there?")

    auth = FaceAuthenticator()

    while not found_user and (time.time() - search_start_time < SEARCH_TIMEOUT):
        user_name = auth.recognize_face()
        if user_name:
            found_user = True
            stop_body()
            speak(f"Oh! Hello {user_name}. It's good to see you again.")
            load_profile(user_name)
            return

        # Not recognised yet - sweep
        turn_body()
        time.sleep(SEARCH_INTERVAL)
        sweep_count += 1

        if sweep_count >= int(360 / (SEARCH_INTERVAL * 30)):
            stop_body()
            speak("I still don't recognise anyone...")
            break

    if not found_user:
        speak("Hello there! I don’t believe we’ve met.")
        rospy.loginfo("[SEARCH] No user identified after timeout.")

def main():
    """ROS Node Entry Point: Initialise publishers and begin face search."""
    global head_pub, voice_pub, motor_pub, search_start_time

    rospy.init_node("startup_search_node")
    head_pub = rospy.Publisher("/miro/command/head", Twist, queue_size=1)
    voice_pub = rospy.Publisher("/miro/command/voice", String, queue_size=1)
    motor_pub = rospy.Publisher("/miro/command/motor", Twist, queue_size=1)

    time.sleep(2)  # Let publishers initialise properly
    search_start_time = time.time()

    search_loop()

    # After completion, signal readiness to other nodes
    rospy.set_param("miro_ready", found_user)
    rospy.spin()

if __name__ == "__main__":
    main()
