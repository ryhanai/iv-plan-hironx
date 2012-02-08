#!/usr/bin/env ipython
# -*- coding: utf-8 -*-

import sys
sys.path.append('../../iv_plan/src')

from set_env import *

from ivutils import *
import RTC
import _GlobalIDL
import math
from numpy import *
from geo import *

##
## RTC-handle
##

from rtc_handle import *

def get_handle(name, nspace):
    return nspace.rtc_handles[name]

def narrow_service(handle, service, port):
    svc = handle.services[service].provided[port]
    svc.narrow_ref(globals())
    return svc

def activate(handles):
    for hdl in handles:
        hdl.activate()

def deactivate(handles):
    for hdl in handles:
        hdl.deactivate()

def connect(handle1, port1, handle2, port2):
    con = IOConnector([handle1.outports[port1], handle2.inports[port2]])
    con.connect()

def disconnect(con):
    con.disconnect()


rtmenv = RtmEnv(sys.argv, [nameserver])
ns = rtmenv.name_space[nameserver]
ns.list_obj()

hpl = get_handle('ArmPlan0.rtc', ns)
hpl.activate()
plsvc = hpl.services['ArmPlanService'].provided['ArmPlanService0']


##
## encoding/decoding for RTM messaging
##

def Pose3DtoFRAME(pose):
    ori = pose.orientation
    pos = pose.position
    return FRAME(xyzabc=[pos.x,pos.y,pos.z,ori.r,ori.p,ori.y])

def FRAMEtoPose3D(frm):
    pos = RTC.Point3D(frm.vec[0], frm.vec[1], frm.vec[2])
    a,b,c = frm.mat.abc()
    ori = RTC.Orientation3D(a, b, c)
    return RTC.Pose3D(ori, pos)

def encode_FRAME(f):
    import operator
    return reduce(operator.__add__, f.mat) + f.vec

def decode_FRAME(ds):
    return FRAME(mat=array(ds[0:9]).reshape(3,3).tolist(), vec=ds[9:])


# objType definition (temporary):
# 1: parts A
# 2: parts B
# 3: pallet pocket P

import re

def approach_test(objType = 1, joints='rarm'):
    ofrm = plsvc.ref.RecognizeParts(objType)
    a = plsvc.ref.GraspPlan(objType, ofrm, False)
    afrm = a[:12]
    gfrm = a[12:24]
    handwidth = a[24]
    if plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
        raw_input('Press any key to reset pose: ')
        plsvc.ref.MoveArm(afrm, handwidth, joints, False, 0.5)
        plsvc.ref.GoPreparePose()
    else:
        raw_input('Cannot reach with %s !' % joints)

def pick_and_place(ofrm, objType = 1, joints='torso_rarm'):
    hand = 'right' if re.sub('torso_', '', joints) == 'rarm' else 'left'
    a = plsvc.ref.GraspPlan(objType, ofrm, False)
    afrm = a[:12]
    gfrm = a[12:24]
    handwidth = a[24]
    if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
        a = plsvc.ref.RequestNext(afrm, gfrm, handwidth) # try next grasp
        afrm = a[:12]
        gfrm = a[12:24]
        handwidth = a[24]
        if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
            print 'Cannot reach with %s !' % joints
            return False

    plsvc.ref.Grab(hand)
    plsvc.ref.MoveArm(afrm, -1, joints, False, 0.5)

    pfrm = plsvc.ref.RecognizePocket(objType) # find a pocket where grasped parts can be put
    a = plsvc.ref.PlacePlan(objType, pfrm)
    afrm = a[:12]
    gfrm = a[12:24]
    plsvc.ref.MoveArm2(afrm, gfrm, handwidth+20, joints)

    plsvc.ref.Release(hand)
    plsvc.ref.MoveArm(afrm, -1, joints, False, 0.5)

def palletize_right():
    def aux(objType):
        while True:
            ofrm = plsvc.ref.RecognizeParts(objType)
            if not ofrm:
                return
            pick_and_place(ofrm, objType)

    aux(1); aux(2)
    plsvc.ref.GoPreparePose()

def pass_left_to_right(objType):
    if objType == 1:
        q_goal = [0, 0, 1.1, 0.15882788821815941, -0.31022021803713384, -1.6186460357518606, -1.1781919825350482,
                  -0.096582871199665632, -0.23461256447573006, -0.13308174660428049, -0.36125317532098755,
                  -1.5033910600950069, 1.3706292369222353, 0.0028576290419997661, 1.7279863708909555,
                  0.8, -0.1, -0.8, 0.1, 0.8, -0.1, -0.8, 0.1]
        handwidth = 38
    else:
        q_goal = [0, 0, 1.1, 0.078806061289114021, -0.33446638930404399, -1.6174062593925511, -1.2074687162189695,
                  -0.018746179622803873, -0.22843791550139492, -0.13308174660428049, -0.36125317532098755,
                  -1.5033910600950069, 1.3706292369222353, 0.0028576290419997661, 1.7279863708909555,
                  0.8, -0.1, -0.8, 0.1, 0.8, -0.1, -0.8, 0.1]
        handwidth = 25
        
    plsvc.ref.Move(q_goal, 'all') # whole body motion
    plsvc.ref.MoveArmRelative(encode_FRAME(FRAME()), handwidth, 'rarm', False, 0.5) # just close the hand
    plsvc.ref.Release('left')
    plsvc.ref.Grab('right')
    plsvc.ref.MoveArmRelative(encode_FRAME(FRAME(xyzabc=[0,-50,0,0,0,0])), -1, 'rarm', False, 0.5)


def pick_pass_and_place(ofrm, objType):
    plsvc.ref.GoPreparePose()
    joints = 'torso_larm'
    hand = 'left'
    a = plsvc.ref.GraspPlan(objType, ofrm, True)
    afrm = a[:12]
    gfrm = a[12:24]
    handwidth = a[24]
    if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
        a = plsvc.ref.RequestNext(afrm, gfrm, handwidth) # try next grasp
        afrm = a[:12]
        gfrm = a[12:24]
        handwidth = a[24]
        if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
            print 'Cannot reach with %s' % joints
            return False

    plsvc.ref.Grab('left')
    pass_left_to_right(objType)

    joints = 'torso_rarm'
    hand = 'right'
    pfrm = plsvc.ref.RecognizePocket(objType)
    a = plsvc.ref.PlacePlan(objType, pfrm)
    afrm = a[:12]
    gfrm = a[12:24]

    if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth+20, joints):
        print 'Cannot reach with %s' % joints
        return False
    
    plsvc.ref.Release(hand)
    plsvc.ref.MoveArm(afrm, -1, joints, False, 0.5)


def palletize():
    def aux(objType):
        while True:
            ofrm = plsvc.ref.RecognizeParts(objType)
            if not ofrm:
                return
            if decode_FRAME(ofrm).vec[1] < 50:
                pick_and_place(ofrm, objType)
            else:
                pick_pass_and_place(ofrm, objType)

    aux(1); aux(2)
    plsvc.ref.GoPreparePose()



colored_print('ofrm = plsvc.ref.RecognizeParts(1)', 'blue')
colored_print('pick_and_place(ofrm)', 'blue')
colored_print('plsvc.ref.ResetWorld()', 'blue')
colored_print('palletize_right()', 'blue')
colored_print('plsvc.ref.ResetWorld()', 'blue')
colored_print('palletize()', 'blue')
