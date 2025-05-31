#!/usr/bin/env python3
import cv2
import os
import time
import rospy
from std_msgs.msg import String
from Team6_work.face_id.simple_facerec import SimpleFacerec

def authenticate_and_publish():
    rospy.init_node('face_id_auth', anonymous=True)
    pub = rospy.Publisher('/face_id/recognised_user', String, queue_size=10)

    sfr = SimpleFacerec()
    sfr.load_encoding_images("Team6_work/face_id/images/")

    cap = cv2.VideoCapture(0)

    attempt_counter = 0
    recognised_counter = 0
    recognised_name = None
    consistent_threshold = 5

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("Failed to grab frame")
                break

            attempt_counter += 1
            rospy.loginfo(f"Attempt {attempt_counter}")

            face_locations, face_names, is_recognised = sfr.detect_known_faces(frame)

            for (top, right, bottom, left), name in zip(face_locations, face_names):
                if name != "Unknown":
                    if name == recognised_name:
                        recognised_counter += 1
                    else:
                        recognised_name = name
                        recognised_counter = 1

                    if recognised_counter >= consistent_threshold:
                        rospy.loginfo(f"Recognised: {name}")
                        pub.publish(name)

                        cv2.putText(frame, name, (left, top - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                        cv2.imshow("Face ID Authentication", frame)

                        time.sleep(5)
                        cap.release()
                        cv2.destroyAllWindows()
                        return
                else:
                    rospy.loginfo("Face not recognised")

                # Draw red box regardless
                cv2.putText(frame, name, (left, top - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            cv2.imshow("Face ID Authentication", frame)

            if cv2.waitKey(1) & 0xFF == ord('q') or attempt_counter >= 100:
                rospy.logwarn("Face not recognised or max attempts reached.")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by ROS shutdown.")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    authenticate_and_publish()
