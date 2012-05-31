# -*- coding: utf-8 -*-

class PortInfo:
    def __init__(self, target, port):
        self.target = target
        self.port = port

class RTCPortInfo(PortInfo):
    def __init__(self, target, port):
        PortInfo.__init__(self, target, port)

class RTCDataPortInfo(RTCPortInfo):
    def __init__(self, target, port):
        RTCPortInfo.__init__(self, target, port)

class RTCServicePortInfo(RTCPortInfo):
    def __init__(self, target, port, service):
        RTCPortInfo.__init__(self, target, port)
        self.service = service

