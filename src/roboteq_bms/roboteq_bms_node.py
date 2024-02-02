#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from roboteq_bms.roboteq_bms_ros import RoboteqBMS


def main():

    rospy.init_node("roboteq_bms")

    rc_node = RoboteqBMS()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
