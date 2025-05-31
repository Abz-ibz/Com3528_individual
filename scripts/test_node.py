#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node('test_node')
    rospy.loginfo("Test node is running.")
    rospy.spin()

if __name__ == '__main__':
    main()
