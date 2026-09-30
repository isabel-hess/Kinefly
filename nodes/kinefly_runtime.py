#!/usr/bin/env python
# kinefly_runtime.py
# The few runtime services the tracking code needs (logging and a clock),
# formerly provided by rospy.  Timestamps are plain floats in seconds.
import sys
import time


def now():
    return time.perf_counter()


def logwarn(msg):
    sys.stderr.write('[WARN] %s\n' % (msg,))
    sys.stderr.flush()


def loginfo(msg):
    sys.stderr.write('[INFO] %s\n' % (msg,))
    sys.stderr.flush()
