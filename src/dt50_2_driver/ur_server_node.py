#!/usr/bin/env python

import rospy
from dt50_2_driver.ur_server import URBridge


def main():

    rospy.init_node("ur_server_node")

    rc_node = URBridge()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()