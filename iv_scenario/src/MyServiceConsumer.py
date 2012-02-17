#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import string

from ivenv import *
import RTC
#import SimpleService
import OpenRTM_aist
from omniORB import CORBA
import _GlobalIDL

myserviceconsumer_spec = ["implementation_id", "MyServiceConsumer",
                          "type_name",         "MyServiceConsumer",
                          "description",       "MyService Consumer Sample component",
                          "version",           "1.0",
                          "vendor",            "Shinji Kurihara",
                          "category",          "example",
                          "activity_type",     "DataFlowComponent",
                          "max_instance",      "10",
                          "language",          "Python",
                          "lang_type",         "script",
                          ""]


class MyServiceConsumer(OpenRTM_aist.DataFlowComponentBase):
  # constructor
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

    self._async_echo = None
    self._result = [None]
    return

  def onInitialize(self):
    # initialization of CORBA Port
    self._myServicePort = OpenRTM_aist.CorbaPort("MyService")

    # initialization of Consumer
    self._myservice0 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.RecognitionService)
        
    # Set service consumers to Ports
    self._myServicePort.registerConsumer("myservice0", "MyService", self._myservice0)

    # Set CORBA Service Ports
    self.addPort(self._myServicePort)

    return RTC.RTC_OK

  # The execution action that is invoked periodically
  def onExecute(self, ec_id):
    print "hit return"
    sys.stdin.readline()
    self._myservice0._ptr().setModelID(1)
    return RTC.RTC_OK


def MyServiceConsumerInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=myserviceconsumer_spec)
  manager.registerFactory(profile,
                          MyServiceConsumer,
                          OpenRTM_aist.Delete)


def MyModuleInit(manager):
  MyServiceConsumerInit(manager)

  # Create a component
  comp = manager.createComponent("MyServiceConsumer")
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
  return


if __name__ == "__main__":
  main()
