# startup_search_node.py
# Controls MiRo's startup face search behavior, recognition, and profile activation

import time
import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from face_id.authentication import recognize_face  # From faceID code

# ROS publishers
head_pub = None
voice_pub = None
motor_pub = None

# Constants
SEARCH_TIMEOUT = 90  # seconds to give up if no known face is found
SEARCH_INTERVAL = 5   # seconds between turns
PROFILE_DIR = "profiles/"

# Global state
found_user = False
user_name = None
search_start_time = None

def speak(text):
    voice_pub.publish(text)

def turn_head(yaw):
    twist = Twist()
    twist.angular.z = yaw  # Simulated head rotation left/right
    head_pub.publish(twist)

def turn_body():
    twist = Twist()
    twist.angular.z = 0.5  # Slow clockwise turn
    motor_pub.publish(twist)

def stop_body():
    twist = Twist()
    twist.angular.z = 0.0
    motor_pub.publish(twist)

def load_profile(name):
    try:
        with open(f"{PROFILE_DIR}{name}.json", "r") as f:
            profile = json.load(f)
        rospy.set_param("miro_active_profile", name)
        rospy.loginfo(f"Loaded profile for {name}")
        return profile
    except FileNotFoundError:
        rospy.logwarn("Profile not found")
        return None

def search_loop():
    global found_user, user_name
    sweep_count = 0
    speak("Hmm... Hello? Anyone there?")

    while not found_user and (time.time() - search_start_time < SEARCH_TIMEOUT):
        user_name = recognize_face()
        if user_name:
            found_user = True
            stop_body()
            speak(f"Oh! Hello {user_name}. It's good to see you again.")
            load_profile(user_name)
            return

        # Not recognized yet - sweep
        turn_body()
        time.sleep(SEARCH_INTERVAL)
        sweep_count += 1

        if sweep_count >= int(360 / (SEARCH_INTERVAL * 30)):  # Simulated full turn
            stop_body()
            speak("I still don't recognise anyone...")
            break

    if not found_user:
        speak("Hello there! I don’t believe we’ve met.")
        # Could optionally prompt for name input here

def main():
    global head_pub, voice_pub, motor_pub, search_start_time

    rospy.init_node("startup_search_node")
    head_pub = rospy.Publisher("/miro/command/head", Twist, queue_size=1)
    voice_pub = rospy.Publisher("/miro/command/voice", String, queue_size=1)
    motor_pub = rospy.Publisher("/miro/command/motor", Twist, queue_size=1)

    time.sleep(2)  # Let publishers register
    search_start_time = time.time()

    search_loop()

    # Transition to main behavior loop (e.g. conversation)
    rospy.set_param("miro_ready", found_user)
    rospy.spin()

if __name__ == "__main__":
    main()
