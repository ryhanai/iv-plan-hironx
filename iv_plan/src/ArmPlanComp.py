#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import string
import time

from set_env import *

##
## Important:
##  RTM-related packages must be imported before those of vpython.
##  Otherwise, SEGV occurs when sending a CORBA message.
##

# from demo_common import *

import RTC
import _GlobalIDL, _GlobalIDL__POA
import OpenRTM_aist

# Module specification
mplanserviceprovider_spec = ["implementation_id", "ArmPlan",
                             "type_name",         "ArmPlan",
                             "description",       "Planner for HIRO-NX",
                             "version",           "0.31.0",
                             "vendor",            "Ryo Hanai",
                             "category",          "Planner",
                             "activity_type",     "EVENTDRIVEN",
                             "kind",              "DataFlowComponent",
                             "max_instance",      "0",
                             "language",          "Python",
                             "lang_type",         "script",
                             ""]


def Pose3DtoFRAME(pose):
    ori = pose.orientation
    pos = pose.position
    return FRAME(xyzabc=[pos.x,pos.y,pos.z,ori.r,ori.p,ori.y])

def FRAMEtoPose3D(frm):
    pos = RTC.Point3D(frm.vec[0], frm.vec[1], frm.vec[2])
    a,b,c = frm.mat.abc()
    ori = RTC.Orientation3D(a, b, c)
    return RTC.Pose3D(ori, pos)

import operator

def encode_FRAME(f):
    return reduce(operator.__add__, f.mat) + f.vec

def decode_FRAME(ds):
    return FRAME(mat=array(ds[0:9]).reshape(3,3).tolist(), vec=ds[9:])

def prefix(objectType):
    table = {1:'A', 2:'B'}
    try:
        return table[objectType]
    except:
        return []


class ArmPlanServiceSVC_impl(_GlobalIDL__POA.ArmMotionService):
    def __init__(self):
        return
    
    def __del__(self):
        pass

    def MoveArm(self, goal, handWidth, joints, checkCollision, duration):
        frm = decode_FRAME(goal)
        if handWidth <= 0.0: # doesn't move gripper
            handWidth = None
        return move_arm(frm,joints=joints, width=handWidth,
                        check_collision=checkCollision, duration=duration)
    
    def MoveArm2(self, afrm, gfrm, handWidth, joints):
        afrm = decode_FRAME(afrm)
        gfrm = decode_FRAME(gfrm)
        return move_arm2(afrm, gfrm, handWidth, joints=joints)

    def Grab(self, hand):
        return grab(hand=hand)

    def Release(self, hand):
        return release(hand=hand)

    def Move(self, goalConfig, joints):
        q0 = r.get_joint_angles(joints=joints)
        print q0
        print goalConfig
        traj = pl.make_plan(q0, goalConfig, joints=joints)
        exec_traj(traj, joints=joints)
        return True

    def MoveArmRelative(self, reltrans, handWidth, joints, checkCollision, duration):
        f = decode_FRAME(reltrans)
        g = r.fk(parse_joints_flag(joints)[0])
        frm = FRAME(mat=g.mat*f.mat, vec=g.vec+f.vec)
        if handWidth <= 0.0:
            handWidth = None
        return move_arm(frm,joints=joints, width=handWidth,
                        check_collision=checkCollision, duration=duration)

    def GoPreparePose(self):
        return go_prepare_pose()

    def GetJointAngles(self, joints):
        return r.get_joint_angles(joints=joints)

    def Fk(self, joints):
        return encode_FRAME(r.fk(parse_joints_flag(joints)[0]))

    def Ik(self, frm, joints):
        return r.ik(decode_FRAME(frm), joints=joints)[0]

    def AddObject(self, name, objectType, worldFrame):
        if objectType == 1:
            obj = env.eval_sctree(scene_objects.partsA(name))
        elif objectType == 2:
            obj = env.eval_sctree(scene_objects.partsB(name))
        else:
            return

        r.add_collision_object(obj)
        env.insert_object(obj, decode_FRAME(worldFrame), parent=env.get_world())

    def DeleteObject(self, name):
        env.delete_object(name)

    def GraspPlan(self, otype, ofrm, longSide):
        ofrm = decode_FRAME(ofrm)
        afrm,gfrm,handwidth = graspplan(otype, ofrm, long_side=longSide)
        return encode_FRAME(afrm) + encode_FRAME(gfrm) + [handwidth]

    def RequestNext(self, afrm, gfrm, handwidth):
        afrm2,gfrm2 = request_next(afrm, gfrm)
        return encode_FRAME(afrm2) + encode_FRAME(gfrm2) + [handwidth]

    def PlacePlan(self, otype, pfrm):
        pfrm = decode_FRAME(pfrm)
        afrm,gfrm = placeplan(otype, pfrm)
        return encode_FRAME(afrm) + encode_FRAME(gfrm)

    def RecognizePocket(self, objectType):
        def occupied(p, otyp):
            eps = 10.0
            return reduce(operator.__or__,
                          [linalg.norm(array(o.where().vec[:2]-array(p.where().vec[:2]))) < eps
                           for o in env.get_objects(prefix(objectType))])

        os = [o for o in env.get_objects('P') if not occupied(o, objectType)]
        if len(os) > 0:
            return encode_FRAME(os[0].where())
        else:
            return []

    def RecognizeParts(self, objectType):
        os = [o for o in env.get_objects(prefix(objectType)) if o.where().vec[1] > -80]

        if len(os) > 0:
            return encode_FRAME(os[0].where())
            #return encode_FRAME(os[random.randint(0,len(os)-1)].where())
        else:
            return []

    def ResetWorld(self):
        reset_parts()
        

class ArmPlanServiceProvider(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        # initialization of CORBA Port
        self._armPlanServicePort = OpenRTM_aist.CorbaPort("ArmPlanService")
        self._seqPlayServicePort = OpenRTM_aist.CorbaPort("SeqPlayService")
        self._hiroControllerServicePort = OpenRTM_aist.CorbaPort("HiroControllerService")
        
        # initialization of Provider
        self._service0 = ArmPlanServiceSVC_impl()

        # initialization of Consumer
        self._service1 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.SequencePlayerService)
        self._service2 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.MotionCommands)
        # Set service providers to Ports
        self._armPlanServicePort.registerProvider("ArmPlanService0",
                                                  "ArmPlanService",
                                                  self._service0)

        # Set service consumers to Ports
        self._seqPlayServicePort.registerConsumer("SeqPlayService0",
                                                  "SeqPlayService",
                                                  self._service1)

        self._hiroControllerServicePort.registerConsumer("HiroControllerService0",
                                                         "HiroControllerService",
                                                         self._service2)

        # Set CORBA Service Ports
        self.addPort(self._armPlanServicePort)
        self.addPort(self._seqPlayServicePort)
        self.addPort(self._hiroControllerServicePort)

        return RTC.RTC_OK


def ArmPlanInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=mplanserviceprovider_spec)
    manager.registerFactory(profile,
                            ArmPlanServiceProvider,
                            OpenRTM_aist.Delete)
    return


def ArmPlanModuleInit(manager):
    ArmPlanInit(manager)
    # Create a component
    comp = manager.createComponent("ArmPlan")
    return


def main():
    # Initialize manager
    mgr = OpenRTM_aist.Manager.init(sys.argv)

    # Set module initialization proceduer
    # This procedure will be invoked in activateManager() function.
    mgr.setModuleInitProc(ArmPlanModuleInit)

    # Activate manager and register to naming service
    mgr.activateManager()

    # run the manager in blocking mode
    # runManager(False) is the default
    mgr.runManager()

    # If you want to run the manager in non-blocking mode, do like this
    # mgr.runManager(True)


if __name__ == "__main__":
    main()
