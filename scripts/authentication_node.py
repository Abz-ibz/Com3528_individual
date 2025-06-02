# authentication_node.py

import rospy
from std_msgs.msg import String
from face_id.authentication import FaceAuthenticator
import logging
from config import (
    DEBUG_FACE_RECOGNITION,
    EXPLORE_MODE,
    REMINISCENCE_MODE
)

# Set up logging level based on debug flag
logging.basicConfig(level=logging.DEBUG if DEBUG_FACE_RECOGNITION else logging.INFO,
                    format='[%(levelname)s] %(message)s')

def main():
    rospy.init_node('face_authentication_node')
    pub = rospy.Publisher('/miro/user_id', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    authenticator = FaceAuthenticator()
    logging.info("Face Authentication Node started. Waiting for user...")

    try:
        while not rospy.is_shutdown():
            user = authenticator.recognize_face()

            if user:
                pub.publish(user)
                logging.info(f"[ROS] Published recognised user: {user}")

                # Optionally, adjust mode flags externally via ROS service or topic
                if REMINISCENCE_MODE:
                    logging.info("System is in REMINISCENCE MODE. Memory saving enabled.")
                elif EXPLORE_MODE:
                    logging.info("System is in EXPLORE MODE. No memory saving will occur.")
            else:
                pub.publish("Unknown")
                logging.debug("[ROS] Published user: Unknown")

            rate.sleep()

    except rospy.ROSInterruptException:
        logging.info("ROS node shutdown requested.")
    finally:
        authenticator.release()
        logging.info("Camera released. Node terminated.")


if __name__ == '__main__':
    main()
