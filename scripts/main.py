#!/usr/bin/python

import moveit_commander
import sys
import rospy


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('gen3_lite_pick_place')
    rospy.sleep(2)


if __name__ == "__main__":
    main()
