# -*- coding: utf-8 -*-

class VPA10(VRobot):
    def __init__(self,
                 wrldir,
                 scale = 1000.0,
                 name = 'pa10'):
        self.poses = { 'reset' : zeros(9),
                       'manip' : array([0,-0.4,0,1.2,0,1,0]),
                       'pickup' : array([1.4,0.3,0,1.2,0,1.6,0]),
                       'undertable' : array([1.5,0.3,0,2.2,0,0.5,0])
                       }

        VRobot.__init__(self, wrldir, scale, name)


class VRH2(VRobot):
    def __init__(self,
                 wrldir,
                 scale = 1000.0,
                 name = 'rh2'):

        self.poses = { 'reset' : zeros(1) }

        VRobot.__init__(self, wrldir, scale, name)


