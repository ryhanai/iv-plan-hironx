#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import string
import time

import roslib; roslib.load_manifest('iv_plan')
import set_env

import RTC
import _GlobalIDL, _GlobalIDL__POA
import OpenRTM_aist

##
## Important:
##  RTM-related packages must be imported before those of vpython.
##  Otherwise, SEGV occurs when sending a CORBA message.
##

from demo_common import *


coordtransserviceprovider_spec = ["implementation_id", "CoordTrans",
                                  "type_name",         "CoordTrans",
                                  "description",       "Coordinates transformer for HIRO-NX",
                                  "version",           "0.31.0",
                                  "vendor",            "Ryo Hanai",
                                  "category",          "Tool",
                                  "activity_type",     "EVENTDRIVEN",
                                  "kind",              "DataFlowComponent",
                                  "max_instance",      "0",
                                  "language",          "Python",
                                  "lang_type",         "script",
                                  ""]


import operator

def encode_FRAME(f):
    m = eye(4)
    m[0:3,0:3] = f.mat
    m[0:3,3] = f.vec
    return m.reshape(16).tolist()

def decode_FRAME(f):
    f = array(f).reshape(4,4)
    return FRAME(mat=f[0:3,0:3].tolist(), vec=f[0:3,3].tolist())

class CoordTransServiceSVC_impl(_GlobalIDL__POA.CoordTransService):
    def __init__(self):
        return

    def __del__(self):
        pass

    def Query(self, sensor, recogframe, robotframe, jointangles):
        Tobj = decode_FRAME(recogframe)
        Trobo = decode_FRAME(robotframe)
        r.locate(Trobo)
        r.set_joint_angles(jointangles)
        res = r.get_sensor(sensor).where() * Tobj
        print res
        show_frame(res)
        return encode_FRAME(res)

class CoordTransServiceProvider(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        self._coordTransServicePort = OpenRTM_aist.CorbaPort("CoordTransService")
        self._service0 = CoordTransServiceSVC_impl()
        self._service1 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.SequencePlayerService)
        self._coordTransServicePort.registerProvider("CoordTransService0",
                                                  "CoordTransService",
                                                  self._service0)
        self.addPort(self._coordTransServicePort)
        return RTC.RTC_OK


def CoordTransInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=coordtransserviceprovider_spec)
    manager.registerFactory(profile,
                            CoordTransServiceProvider,
                            OpenRTM_aist.Delete)
    return


def CoordTransModuleInit(manager):
    CoordTransInit(manager)
    comp = manager.createComponent("CoordTrans")
    return


def main():
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(CoordTransModuleInit)
    mgr.activateManager()
    mgr.runManager()


if __name__ == "__main__":
    main()
