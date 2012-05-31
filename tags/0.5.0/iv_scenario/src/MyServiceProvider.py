#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import string
import time
import operator

from demo_wexpo import *

import RTC
import _GlobalIDL__POA
import OpenRTM_aist
import _GlobalIDL


def toFloatArray(f):
  a = numpy.zeros((3,4))
  a[:,:3] = f.mat
  a[:,3] = f.vec
  res = zeros(20)
  res[8:] = a.flatten()
  return res.tolist()


# Module specification
myserviceprovider_spec = ["implementation_id", "MyServiceProvider",
                          "type_name",         "MyServiceProvider",
                          "description",       "MyService Provider Sample component",
                          "version",           "1.0",
                          "vendor",            "Shinji Kurihara",
                          "category",          "example",
                          "activity_type",     "DataFlowComponent",
                          "max_instance",      "10",
                          "language",          "Python",
                          "lang_type",         "script",
                          ""]


# Class implementing IDL interface MyService(MyService.idl)
class MyServiceSVC_impl(_GlobalIDL__POA.RecognitionService):
  def __init__(self):
    self.modelID = 1
    self._frames = []
    return

  def __del__(self):
    pass

  def getModelID(self):
    return self.modelID

  def setModelID(self, modelID):
    print 'modelID: ', modelID
    self.modelID = modelID

    try:
      # do scan the table
      fs = look_for()

      # just return the frames in the simulator
      # fs = [o.where() for o in env.get_objects('^A.*$')]
      self._frames = reduce(operator.__add__, [toFloatArray(f) for f in fs])
    except:
      print 'recognition failed'

class MyServiceProvider(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return


  def onInitialize(self):
    # initialization of CORBA Port
    self._myServicePort = OpenRTM_aist.CorbaPort("MyService")

    # initialization of Provider
    self._myservice0 = MyServiceSVC_impl()

    # Set service providers to Ports
    self._myServicePort.registerProvider("myservice0", "MyService", self._myservice0)

    # Set CORBA Service Ports
    self.addPort(self._myServicePort)


    self._result = RTC.TimedDoubleSeq(RTC.Time(0,0),[1,2,3])
    self._resultOut = OpenRTM_aist.OutPort("RecognitionResult", self._result)
    self.addOutPort("RecognitionResult", self._resultOut)

    return RTC.RTC_OK


  def onExecute(self, ec_id):
    if len(self._myservice0._frames) > 0:
        self._result.data = self._myservice0._frames
        self._resultOut.write()
        self._myservice0._frames = []

    time.sleep(0.5)
    return RTC.RTC_OK

def MyServiceProviderInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=myserviceprovider_spec)
  manager.registerFactory(profile,
                          MyServiceProvider,
                          OpenRTM_aist.Delete)
  return


def MyModuleInit(manager):
  MyServiceProviderInit(manager)

  # Create a component
  comp = manager.createComponent("MyServiceProvider")

  """
  rtobj = manager.getPOA().servant_to_reference(comp)._narrow(RTC.RTObject)

  ecs = rtobj.get_execution_context_services()
  ecs[0].activate_component(rtobj)
  """
  return


def main():
  # Initialize manager
  mgr = OpenRTM_aist.Manager.init(sys.argv)

  # Set module initialization proceduer
  # This procedure will be invoked in activateManager() function.
  mgr.setModuleInitProc(MyModuleInit)

  # Activate manager and register to naming service
  mgr.activateManager()

  # run the manager in blocking mode
  # runManager(False) is the default
  mgr.runManager()

  # If you want to run the manager in non-blocking mode, do like this
  # mgr.runManager(True)


if __name__ == "__main__":
  main()
