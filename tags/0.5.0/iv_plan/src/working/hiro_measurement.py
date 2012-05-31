# -*- coding: utf-8 -*-

from numpy import *
import sys
import time

from demo_common import *
from setup_rtchandle import *

profile = { 0: [pi/4,0],
            1: [pi/4,0],
            2: [pi/4,0],
            3: [pi/4,0],
            4: [-pi/4,0],
            5: [-pi/4,0],
            6: [pi/4,0],
            7: [pi/4,0],
            8: [pi/4,0] }

def test(jointID=3, duration=2.0):
    r.reset_pose()
    theta0 = r.get_joint_angle(jointID)
    sync(duration=2.0)
    angles = profile[jointID]
    for angle in angles:
        r.set_joint_angle(jointID, theta0 + angle)
        sync(duration=duration)
        time.sleep(0.2)
