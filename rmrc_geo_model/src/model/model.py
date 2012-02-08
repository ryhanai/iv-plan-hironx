#!/usr/bin/env python
# -*- Python -*-

MyRtcDir = "/home/rtc-center/openrtm/myRTC"

import sys

save_path = sys.path[:]
sys.path.append(MyRtcDir+'/tools/geo')
from geo import *
sys.path = save_path

#
# geometric scene
#
class GeoScene :
    def __init__(self) :
        self.parts_list = []
        self.view_camera = []
    def put(self, part) :
        self.parts_list.append(part)
    def rem(self, part) :
        self.parts_list.remove(part)
    def set_viewer(self, vw = None) :
        if vw :
            self.view_camera.append(vw)
    def draw(self) :
        pass
#
# world model
#
class CoordinateObject :
    def __init__(self, trans, prnt = None, rl = None) :
        self.rel_trans = trans
        self.parent = prnt
        self.rel = rl
    def where(self) :
        if self.parent :
            return self.parent.where() * self.rel_trans
        else :
            return self.rel_trans
    def affix(self, mama, re=None) :
        if self.parent :
            print "error! no more parent."
        else :
            self.rel_trans = (- mama.where()) * self.where()
            self.parent = mama
            self.rel = re
    def unfix(self) :
        self.rel_trans = self.where()
        self.parent = None
        self.rel = None
    def set_pos(self, pos) :
        if self.rel :
            print "error! cannot set_pos."
        elif self.parent :
            self.rel_trans = ( - self.parent.where()) * pos
        else :
            self.rel_trans = pos
#
# parts
#
class PartsObejct :
    pass
#
# Actuator
#
class Actuator :
    def __init__(self) :
        self.base = CoordinateObejct(FRAME())
        self.ef = CoordinateObject(FRAME(),self.base)
        self.affix(self.base)
    def move(self, trans) :
        self.ef.rel_trans = trans
