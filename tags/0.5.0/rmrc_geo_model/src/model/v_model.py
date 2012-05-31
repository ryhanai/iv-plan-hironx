#!/usr/bin/env python
# -*- Python -*-

MyRtcDir = "/home/rtc-center/openrtm/myRTC"

import sys
import new
import visual

save_path = sys.path[:]
sys.path.append(MyRtcDir+'/tools/geo')
from geo import *
sys.path.append(MyRtcDir+'/tools/model')
from model import *
sys.path = save_path

#
# world model
#
class V_CoordinateObject(CoordinateObject, visual.frame) :
    def __init__(self, trans, prnt = None, rl = None) :
        CoordinateObject.__init__(self,trans,prnt,rl)
        visual.frame.__init__(self)
        self.frame = prnt
#        self.x_arrow = visual.arrow(axis=(0.005,0,0),shaftwidth=0.001)
#        self.x_arrow.frame=self
#        self.y_arrow = visual.arrow(axis=(0,0.005,0),shaftwidth=0.001)
#        self.y_arrow.frame=self
#        self.z_arrow = visual.arrow(axis=(0,0,0.005),shaftwidth=0.001)
#        self.z_arrow.frame=self
    def affix(self, mama, re=None) :
        if self.parent :
            print "error! no more parent."
        else :
            CoordinateObject.affix(self, mama, re)
            self.frame = mama
    def unfix(self) :
        CoordinateObject.unfix(self)
        self.frame = None
# set_posは、world座標内でのオブジェクトの移動、親に固定されていると動
#かない。
    def set_pos(self, pos) :
        if self.rel :
            print "error! cannot set_pos."
        elif self.parent :
            old_trans = self.rel_trans
            self.rel_trans = ( - self.parent.where()) * pos
            diff_trans = self.rel_trans * ( - old_trans)
            rot = diff_trans.mat.rot_axis()
            self.pos = self.rel_trans.vec.val
            self.rotate(axis=rot[1].val,angle=rot[0])
        else :
            old_trans = self.rel_trans
            self.rel_trans = pos
            diff_trans = self.rel_trans * ( - old_trans)
            rot = diff_trans.mat.rot_axis()
            self.pos = self.rel_trans.vec.val
            self.rotate(axis=rot[1].val,angle=rot[0])
# moveは、相対座標内でのオブジェクトの移動、親に固定されていてもそれが
#変化する。
    def move(self, pos) :
        old_trans = self.rel_trans
        self.rel_trans = pos
        diff_trans = self.rel_trans * ( - old_trans)
        rot = diff_trans.mat.rot_axis()
        self.pos = self.rel_trans.vec.val
        self.rotate(axis=rot[1].val,angle=rot[0])
#
# visual parts, 
#
class V_PartsObject(V_CoordinateObject) :
    def __init__(self, trans, prnt = None, rl = None, shape=None) :
        V_CoordinateObject.__init__(self,trans,prnt,rl)
        if shape :
            self.set_shape(shape)
    def set_shape(self,shape) :
        self.shape = shape
        self.shape.frame = self
#
# visual Actuator
#
class V_Actuator(V_CoordinateObject) :
    def __init__(self, trans, prnt = None, rl = None, shape=None) :
        V_CoordinateObject.__init__(self,trans,prnt,rl)
    def set_base(self, base) :
        self.affix(base)
    def move(self, trans) :
        self.set_pos
        self.ef.rel_trans = trans
