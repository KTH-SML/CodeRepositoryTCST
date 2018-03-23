#!/usr/bin/env python
# -*- coding: utf-8 -*-

from StringIO import StringIO

import rospy
from std_srvs.srvs import Empty

from rqt_simulation._move_base_wrapper_cpp import MoveBaseWrapper

class MoveBase(object):
    def __init__(self):
        self._move_base = MoveBaseWrapper()

    def _to_cpp(self, msg):
        buf = StringIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        msg = cls()
        return msg.deserialize(str_msg)

    def clearCostmapsService(self, req, resp):
        if not isinstance(req, Empty.Request):
            rospy.ROSExcetption('Argument 1 is not a std_srvs/Empty/Request')
        if not isinstance(req, Empty.Response):
            rospy.ROSExcetption('Argument 1 is not a std_srvs/Empty/Response')
        str_req = self._to_cpp(req)
        str_resp = self._to_cpp(resp)
        return self._from_cpp(True)

