# authentication_node.py

import rospy
from std_msgs.msg import String
from authentication import FaceAuthenticator
import time
import logging
from config import DEBUG_FACE_RECOGNITION

# Configure logging with debug control from config
logging.basicConfig(level=logging.DEBUG if DEBUG_FACE_RECOGNITION else logging.INFO,
                    format='[%(levelname)s] %(message)s')

def main():
    """
    ROS node for real-time face authentication. Publishes the name of recognised users to a topic.
    This node continuously checks the video stream for known users and publishes the result to /miro/user_identified.
    """
    rospy.init_node("face_authentication_node")  # Initialise ROS node
    user_pub = rospy.Publisher("/miro/user_identified", String, queue_size=10)  # Topic to publish user identity
    rate = rospy.Rate(1)  # 1 Hz polling rate

    auth = FaceAuthenticator()  # Initialise face authentication wrapper

    logging.info("[ROS] Face authentication node started. Scanning for known users...")

    try:
        while not rospy.is_shutdown():
            user = auth.recognize_face()  # Try to recognise a face
            if user:
                logging.info(f"[ROS] User recognized: {user}")
                user_pub.publish(user)  # Publish recognised username to ROS
                time.sleep(3)  # Wait before next attempt to avoid spam and flicker
            rate.sleep()

    except rospy.ROSInterruptException:
        logging.info("[ROS] Face authentication node interrupted by system shutdown.")

    finally:
        auth.release()  # Ensure camera is released
        logging.info("[ROS] Camera released. Node shutdown complete.")

if __name__ == "__main__":
    main()
