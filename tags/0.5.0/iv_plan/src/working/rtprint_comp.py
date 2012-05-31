#!/usr/bin/env python
# -*- Python -*-
# -*- coding: utf-8 -*-

'''rtshell

Copyright (C) 2009-2011
    Geoffrey Biggs
    RT-Synthesis Research Group
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt

Reader component used by rtprint

'''


from rtshell import gen_comp
import OpenRTM_aist
import RTC
import time

###############################################################################
## Reader component for rtprint

class Reader(gen_comp.GenComp):
    def __init__(self, mgr, port_specs, *args, **kwargs):
        gen_comp.GenComp.__init__(self, mgr, port_specs, *args, **kwargs)
        self.buf = [None for ps in port_specs]

    def _behv(self, ec_id):
        execed = 0
        for i, p in enumerate(self._ports.values()):
            if p.port.isNew():
                execed = 1
                p.read()
                self.buf[i] = p.data
                #self.buf.append(p.data)
                #print p.format()
        return RTC.RTC_OK, execed

    def get(self):
        return self.buf
        # if len(self.buf) > 0:
        #     return self.buf.pop(0)
        # else:
        #     return None
