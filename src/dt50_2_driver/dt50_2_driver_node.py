#!/usr/bin/env python3

import rospy
from dt50_2_driver import DT502Driver


def main():

    rospy.init_node("dt50_2_driver_node")

    rc_node = DT502Driver()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()