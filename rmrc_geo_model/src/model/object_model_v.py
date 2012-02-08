#!/usr/bin/env python
# -*- Python -*-

MyToolDir = ".."

import visual
import sys
import time

save_path = sys.path[:]
sys.path.append(MyToolDir+'/geo')
from geo import *
sys.path = save_path

from geo import *

#
# axis
#
class AxesXYZ(visual.frame) :
    d_visible=True
    d_size=0.1
    members=[]
    def __init__(self, size=None) :
        visual.frame.__init__(self)
        if not size :
            size=AxesXYZ.d_size
        self.x_axis=visual.arrow(axis=(size,0.0,0.0))
        self.x_axis.color=visual.color.red
        self.x_axis.frame=self
        self.y_axis=visual.arrow(axis=(0.0,size,0.0))
        self.y_axis.color=visual.color.green
        self.y_axis.frame=self
        self.z_axis=visual.arrow(axis=(0.0,0.0,size))
        self.z_axis.color=visual.color.blue
        self.z_axis.frame=self
        self.visible(AxesXYZ.d_visible)
        AxesXYZ.members.append(self)
    def visible(self,vv=1) :
        self.x_axis.visible=vv
        self.y_axis.visible=vv
        self.z_axis.visible=vv
    def resize(self,size=0.1) :
        self.x_axis.length=size
        self.y_axis.length=size
        self.z_axis.length=size
    @classmethod
    def visible_all(cls,vv=1):
        cls.d_visible=vv
        for mm in cls.members :
            mm.visible(vv)
    @classmethod
    def resize_all(cls,size=0.1):
        cls.d_size=size
        for mm in cls.members :
            mm.resize(size)           
#
# world model
#
class CoordinateObject :
    def __init__(self) :
        self.parent = None
        self.rel = None
        self.rel_trans = FRAME()
        self.vframe=AxesXYZ()
        
    def where(self,wrt=None) :
        if self.parent :
            my_trans= self.parent.where() * self.rel_trans
        else :
            my_trans= self.rel_trans
        if wrt :
            wrt_trans=wrt.where()
            my_trans =(- wrt_trans) * my_trans
        return my_trans

    def affix(self, mama, rel_trans=None, rel=None) :
        if self.parent :
            print "error! no more parent."
        else :
            if rel_trans:
                self.set_trans(rel_trans)
            else :
                self.set_trans(self.where(mama))
            self.parent = mama
            self.vframe.frame = mama.vframe
            self.rel = rel
    def unfix(self) :
        tmp = self.where()
        self.parent = None
        #self.vframe.frame = None
        self.rel = None
        self.set_trans(tmp)
    def set_pos(self, pos) :
        if self.rel :
            print "error! cannot set_pos."
        elif self.parent :
            tmp =( - self.parent.where()) * pos
            self.set_trans(tmp)     
        else :
            self.set_trans(pos)
    def set_trans(self, trans) :
            tmp = trans
            tmp_f = (- self.rel_trans) * tmp

            tmp_axis=tmp_f.mat.rot_axis()
            tmp_axis[1]= self.rel_trans.mat * tmp_axis[1]
            self.vframe.rotate(axis=tmp_axis[1],angle=tmp_axis[0])
            self.vframe.pos=visual.vector(tmp.vec)
            self.rel_trans = tmp                     
    def ancestor(self, coord) :
        if self == coord :
            return True
        elif not self.parent :
            return False
        elif (coord == self.parent) :
            return True
        else :
            return self.parent.ancestor(coord)
#
# parts
#
class PartsObject(CoordinateObject) :
    def __init__(self, vbody=None) :
        CoordinateObject.__init__(self)
        if vbody :
            self.set_vbody(vbody)
        self.grip=Grip(self)
    def set_vbody(self, vbody):
        self.vbody = vbody
        self.vbody.frame = self.vframe
    
#
class Grip(CoordinateObject) :
    def __init__(self, mama, trans=None) :
        CoordinateObject.__init__(self)
        self.app_pos=CoordinateObject()
        self.app_pos.affix(self,FRAME(xyzabc=[0,0,0.05,0,0,0]))   #default
        self.width = 0.0
        if not trans :
            trans = FRAME()
        self.affix(mama,trans)
#
# arm:virtual class for robot arm
#
class ArmWithHand :
    def __init__(self, base_pos=None,hand=None,init_pos=None) :
        self.base = PartsObject()
        if base_pos :
            self.base.set_pos(base_pos)
        if not init_pos :
            init_pos=FRAME()
        self.init_pos=init_pos
        self.wrist = PartsObject()
        self.wrist.affix(self.base,init_pos,'rigid')
        self.tools={}     
        self.hand=hand
        if hand :
            self.set_hand(hand)
    def set_hand(self, hand):
        self.hand=hand
        self.tools['hand']=hand
        hand.affix(self.wrist,hand.rel_trans,'rigid')
    def move(self, smthng, smwhr, wrt=None) :
        if smthng.__class__ == str :
           smthng = self.tools[smthng]
        if smthng.ancestor(self.wrist) :
           if isinstance(smwhr,CoordinateObject) :
              target=smwhr.where(self.base)
           elif smwhr.__class__ == FRAME :
              if wrt :
                 target = wrt.where(self.base) * smwhr
              else :
                 target = (- self.base.where()) * smwhr
           else :
              print "error: wrong place expression"
              return False
           self.wrist.set_trans(target*(- smthng.where(self.wrist)))
           time.sleep(1)
        else :
          print "error: cannot move unfixed object"
          return False
#
# hand:virtual class for robot hand
#
class Hand(PartsObject) :
    def __init__(self, trans=None) :
        if not trans :
            trans=FRAME()
        PartsObject.__init__(self)
        if trans :
            self.set_pos(trans)
        self.width=0.0
    def open(self, width) :
        self.widht = width
        time.sleep(1)
        print "open"
    def close(self, width) :
        self.widht = width
        time.sleep(1)
        print "close"
    def width(self) :
        return self.width
