#!/usr/bin/env python3

import rospy
import logging
from std_msgs.msg import Float64
from com3528_individual.msg import FaceData
from geometry_msgs.msg import Twist

import sys
import os

# Add project base directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from settings.config import (
    FRAME_WIDTH,
    CENTERING_GAIN,
    MAX_HEAD_YAW,
    DEBUG_FACE_RECOGNITION
)
from settings.debug_utils import log_info, log_warning, log_error, log_face_recognition

# === ROS Publishers ===
head_yaw_pub = None
motor_pub = None

# === Constants ===
CENTER_TOLERANCE = 40  # pixels of tolerance for being "centered"

# === State Variables ===
last_seen_time = None
centered_once = False

def face_callback(msg):
    global last_seen_time, centered_once

    try:
        x_center = msg.x + msg.width // 2
        frame_center = FRAME_WIDTH // 2
        offset = x_center - frame_center

        head_yaw = -offset * CENTERING_GAIN
        head_yaw = max(-MAX_HEAD_YAW, min(MAX_HEAD_YAW, head_yaw))

        if DEBUG_FACE_RECOGNITION:
            log_face_recognition(f"Offset: {offset}, Commanded Yaw: {head_yaw:.2f}")

        # Check if face is within centering tolerance
        is_centered = abs(offset) <= CENTER_TOLERANCE
        rospy.set_param("miro_face_centered", is_centered)

        if is_centered and not centered_once:
            log_info("Face centered for the first time.")
            centered_once = True

        # Publish the yaw movement to head
        head_yaw_msg = Float64()
        head_yaw_msg.data = head_yaw
        head_yaw_pub.publish(head_yaw_msg)

        # Lock the wheels for stability
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        motor_pub.publish(twist)

        last_seen_time = rospy.Time.now()

    except Exception as e:
        log_error(f"Error in face_callback: {e}")

def main():
    global head_yaw_pub, motor_pub

    logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] %(message)s', force=True)
    rospy.init_node("face_centering_node")

    log_info("Face Centering Node started.")

    try:
        head_yaw_pub = rospy.Publisher("/miro/command/head_yaw", Float64, queue_size=1)
        motor_pub = rospy.Publisher("/miro/command/velocity", Twist, queue_size=1)

        rospy.Subscriber("/face_id/face_data", FaceData, face_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        log_warning("Face Centering Node interrupted.")

if __name__ == '__main__':
    main()
