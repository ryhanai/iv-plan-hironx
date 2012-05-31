# -*- coding: utf-8 -*-

import sys
import os
import time
import random

#from set_env import *
from geo import *
from model import *


class CoordinateObjectWithName(CoordinateObject):
    def __init__(self, name=None):
        CoordinateObject.__init__(self)
        self.name = name
        self.vframe.resize(20.0) # make the arrows visible
        self.children = []

    def __repr__(self):
        return '<%s %s>'%(self.__class__, self.name)

    def __str__(self):
        return self.__repr__()

    def __del__(self):
        #CoordinateObject.__del__(self)
        self.vframe.set_visible(False)

    def locate(self, frame, world=True):
        if world:
            self.set_trans((-self.parent.where())*frame)
        else:
            self.set_trans(frame)

    def set_visible(self, true_or_false):
        self.vbody.vframe.set_visible(true_or_false)

    def trace(self):
        pass

class KinbodyObject(CoordinateObjectWithName):
    def __init__(self, vbody=None, name=None) :
        CoordinateObjectWithName.__init__(self, name)
        if vbody:
            self.set_vbody(vbody)

        self.cb = None
        self.vframe.resize(1.0)

    def set_vbody(self, vbody):
        self.vbody = vbody
        self.vbody.frame = self.vframe

# CoordinateObject
# -> CoordinateObjectWithName
#   -> KinbodyObject
#   -> PartsObjectWithName
#    (-> LinkObject) 
#     -> SensorObject
#   -> JointObject

class PartsObjectWithName(CoordinateObjectWithName):
    def __init__(self, vbody=None, name=None):
        CoordinateObjectWithName.__init__(self, name)
        if vbody :
            self.set_vbody(vbody)
        
        self.name = name
        self.cb = None
        self.vframe.resize(20.0)

    def set_vbody(self, vbody):
        self.vbody = vbody
        self.vbody.frame = self.vframe

    def __repr__(self):
        return '<%s %s>'%(self.__class__, self.name)

    def __str__(self):
        return self.__repr__()

    def locate(self, frame, world=True):
        if world:
            self.set_trans((-self.parent.where())*frame)
        else:
            self.set_trans(frame)


class SensorObject(PartsObjectWithName):
    def __init__(self,
                 name,
                 vbody=visual.box(length=10,height=10,width=10,
                                  color=[0,0,1])
                 ):
        PartsObjectWithName.__init__(self, vbody, name)

    def read(self):
        return FRAME()

class LinkObject:
    def __init__(self, attached_joint, name=None, shapes=[]):
        # CoordinateObject.__init__(self)
        self.attached_joint = attached_joint
        self.name = name
        self.shapes = shapes
        self.cb = None
        self.children = []

    def __repr__(self):
        return '<Link %s>'%self.name

    def __str__(self):
        return self.__repr__()

    def where(self):
        return self.attached_joint.where()

    # def set_trans(self, frm):
    #     self.attached_joint.set_trans(frm)

    # def affix(self, parent, frm):
    #     self.attached_joint.affix(parent, frm)


class JointObject(CoordinateObjectWithName):
    def __init__(self, id_, name, jaxis, reltrans,
                 ulimit=0, llimit=0, uvlimit=0, lvlimit=0):
        CoordinateObjectWithName.__init__(self, name)
        self.id = id_
        self.jaxis = jaxis
        self.reltrans = reltrans
        self.ulimit = ulimit
        self.llimit = llimit
        self.uvlimit = uvlimit
        self.lvlimit = lvlimit
        self.vframe.resize(20) # make the arrow visible
        self.link = None
        self.angle = 0.0

    def __repr__(self):
        if self.id == None:
            return '<Joint %s>'%self.name
        else:
            return '<Joint %s: ID=%d, angle=%f>'%(self.name, self.id, self.angle)

    def __str__(self):
        return self.__repr__()

class CoordinateObjects(CoordinateObjectWithName):
    def __init__(self, name=None):
        CoordinateObjectWithName.__init__(self)
        self.name = name
        self.coords = []

    def append(self, frame):
        co = CoordinateObjectWithName()
        co.vframe.resize(30)
        co.affix(self, frame)
        self.coords.append(co)

    def set_visible(self, true_or_false):
        for co in self.coords:
            co.vframe.set_visible(False)


##
## Writing a robot specific viewer is not a good idea.
## However, separation of robot structures from the viewer is difficult
## since they are tightly coupled in v_robot.
##
class VPA10:
    def __init__(self):
        self.links = [link1, link2, link3, link4, link5, link6, link7]

    def set_joint_angles(self, ths):
        for (link, th) in zip (self.links, ths):
            link.set_joint(th)

    def get_joint_angles(self):
        return map (lambda link: link.angle, self.links)


        

##
## Drawable objects
##

class VObj(visual.frame):
    def __init__(self):
        visual.frame.__init__(self)

    def post_bodyinit(self, frame=None):
        if frame:
            self.set_frame(frame)
        self.set_visible(False)

    def set_visible(self, visible=True):
        for b in self.bodies:
            b.visible = visible

    def set_frame(self, frame):
        mat = frame.mat
        th, ax = mat.rot_axis()
        self.pos = frame.vec
        self.rotate(axis=ax, angle=th)

    def position(self):
        return VECTOR(vec=self.pos)

    def locate(self, pos):
        self.pos = visual.vector(pos)

    def delete(self):
        self.visible = False
        # memory is reclaimed by GC


class VFrame(VObj):
    def __init__(self, frame=None, size = 150.0, width = 0.01):
        VObj.__init__(self)
        self.arrsize = size
        self.bodies = [visual.arrow(pos=(0,0,0), axis=(self.arrsize,0,0),
                                    shaftwidth=width, color=(1,0,0), frame=self),
                       visual.arrow(pos=(0,0,0), axis=(0,self.arrsize,0),
                                    shaftwidth=width, color=(0,1,0), frame=self),
                       visual.arrow(pos=(0,0,0), axis=(0,0,self.arrsize),
                                    shaftwidth=width, color=(0,0,1), frame=self)]
        VObj.post_bodyinit(self, frame)
        
        
class PointCloud(VObj):
    def __init__(self, points, frame=None):
        VObj.__init__(self)
        self.bodies = [visual.points(pos=points, size=3, color=visual.color.blue)]
        VObj.post_bodyinit(self, frame)


class FrameSet(VObj):
    def __init__(self, frames, frame=None):
        VObj.__init__(self)
        self.bodies = []
        for frm in frames:
            print frm
            self.bodies.append(VFrame(frm))
        VObj.post_bodyinit(self, frame)

    def set_visible(self, visible=True):
        for b in self.bodies:
            b.set_visible(visible)


visibleObjects = []

def objects(vobjs=None):
    global visibleObjects
    if not vobjs:
        return visibleObjects
    else:
        if type(vobjs) == list:
            for vobj in visibleObjects:
                vobj.set_visible(False)
            visibleObjects = []
            for vobj in vobjs:
                vobj.set_visible(True)
        else:
            visibleObjects.append(vobjs)
            vobjs.set_visible(True)


def set_view(camera='world'):
    warn('not yet implemented')
    return
    if camera == 'world':
        pass
    elif camera == 'leye':
        pass



##
## viewer settings
## 
visual.scene.center = [0,0,900.0]
visual.scene.forward = [-0.8,0.5,-0.22]
visual.scene.up = [0,0,1]
visual.scene.show_rendertime = True
visual.scene.ambient = visual.color.gray(0.4)
# # visual.scene.autocenter = True
# # visual.scene.objects # a list of all the visible objects in the display
# # visual.scene.mouse.camera.rotate
# # visual.scene.mouse.camera.pos
