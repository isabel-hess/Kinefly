#!/usr/bin/env python
# kinefly_msgs.py
# Plain-Python equivalents of the Kinefly ROS messages (msg/MsgState.msg,
# msg/MsgFlystate.msg) and the SrvTrackerdata response.


class Header(object):
    def __init__(self, seq=0, stamp=0.0, frame_id=''):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id


class MsgState(object):
    def __init__(self):
        self.angles = []
        self.gradients = []
        self.radii = []
        self.freq = 0.0
        self.intensity = 0.0


class MsgFlystate(object):
    def __init__(self):
        self.header = Header()
        self.head = MsgState()
        self.abdomen = MsgState()
        self.left = MsgState()
        self.right = MsgState()
        self.aux = MsgState()


class SrvTrackerdataResponse(object):
    def __init__(self, color='', title1='', title2='', abscissa=(), intensities=(), diffs=(), markersH=(), markersV=()):
        self.color = color
        self.title1 = title1
        self.title2 = title2
        self.abscissa = list(abscissa)
        self.intensities = list(intensities)
        self.diffs = list(diffs)
        self.markersH = list(markersH)
        self.markersV = list(markersV)
