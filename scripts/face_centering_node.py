#!/usr/bin/env python3

import rospy
import traceback
from com3528_individual.msg import FaceData
from std_msgs.msg import Bool
from settings.debug_utils import log_info, log_error, log_warning

class FaceCenteringNode:
    def __init__(self):
        rospy.init_node('face_centering_node')

        # Subscriptions to FaceData from left and right eye nodes
        self.subscriber_left = rospy.Subscriber(
            '/miro/face_id_left', FaceData, self.left_callback)
        self.subscriber_right = rospy.Subscriber(
            '/miro/face_id_right', FaceData, self.right_callback)
        self.publisher = rospy.Publisher(
            '/miro/face_centered', Bool, queue_size=10)

        self.left_data = None
        self.right_data = None

        log_info("[BOOT - CENTERING] Node initialized and subscribers connected.")

    def left_callback(self, msg):
        try:
            self.left_data = msg
            self.evaluate_centering()
        except Exception as e:
            log_error(f"[LEFT CALLBACK ERROR - CENTERING] {e}")
            traceback.print_exc()

    def right_callback(self, msg):
        try:
            self.right_data = msg
            self.evaluate_centering()
        except Exception as e:
            log_error(f"[RIGHT CALLBACK ERROR - CENTERING] {e}")
            traceback.print_exc()

    def evaluate_centering(self):
        try:
            if self.left_data and self.right_data:
                # Both eyes must detect a centered face to confirm centering
                is_centered = self.left_data.is_centered and self.right_data.is_centered
                self.publisher.publish(Bool(data=is_centered))
                log_info(f"[EVAL - CENTERING] Published is_centered = {is_centered}")
        except Exception as e:
            log_error(f"[EVAL ERROR - CENTERING] {e}")
            traceback.print_exc()

if __name__ == '__main__':
    try:
        log_info("[BOOT - CENTERING] Starting face centering evaluation node...")
        FaceCenteringNode()
        rospy.spin()
    except Exception as e:
        log_error(f"[FATAL INIT ERROR - CENTERING] {e}")
        traceback.print_exc()
