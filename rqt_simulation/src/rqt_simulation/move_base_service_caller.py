#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

class MoveBaseServiceCaller(object):
    def __init__(self):
        self.sub_clear_costmap = rospy.Subscriber(self.clear_costmap_topic,
