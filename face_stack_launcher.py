#!/usr/bin/env python3

import subprocess
import time
import rospy
from settings.debug_utils import log_info

def main():
    rospy.init_node("face_stack_launcher", anonymous=False)
    log_info("[LAUNCHER] Starting FaceID Tracking System...")

    time.sleep(1)

    # Launch each FaceID component as subprocess (can also be launched via roslaunch)
    subprocess.Popen(["rosrun", "com3528_individual", "face_id_node.py"])
    subprocess.Popen(["rosrun", "com3528_individual", "face_track_node.py"])
    subprocess.Popen(["rosrun", "com3528_individual", "face_centering_node.py"])

    log_info("[LAUNCHER] All FaceID nodes launched.")
    rospy.spin()

if __name__ == "__main__":
    main()
