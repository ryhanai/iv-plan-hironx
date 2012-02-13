# -*- coding: utf-8 -*-

# Note that RTM nameserver is read from rtc.conf file in the current directory.

from numpy import *
from ivenv import *
from ivutils import *
import hironx_params
import hironxsys
import rtc_helper
import time
import RTC


# Define the modules used in this application, which should be running
# in contexts depending on the environment this program is executed.

portdefs = {'controller' :
                hironx_params.RTCServicePortInfo(target = 'VisionPC.host_cxt/HiroNXProvider0.rtc',
                                                 service = 'HiroNXProvider0.HIRO',
                                                 port = 'motion'),
            'jointstat' :
                hironx_params.RTCDataPortInfo(target = 'RobotHardware0.rtc',
                                              port = 'jointDatOut'),
            'lhandrecog' :
                hironx_params.RTCDataPortInfo(target = 'lhand_cxt/AppRecog0.rtc',
                                              port = 'RecognitionResult'),
            'rhandrecog' :
                hironx_params.RTCDataPortInfo(target = 'rhand_cxt/AppRecog0.rtc', port = 'RecognitionResult')
            }


# Define the interface for my own modules by extending the class for
# HiroNx internal modules.

class MyHiroNxSystem(hironxsys.HiroNxSystem):
    def connect(self):
        try:
            portdef = self.portdefs['rhandrecog']
            self.hrhandrecog = rtc_helper.get_port(portdef)
        except:
            warn('recognition module is not running in rhand')
        try:
            portdef = self.portdefs['lhandrecog']
            self.hlhandrecog = rtc_helper.get_port(portdef)
        except:
            warn('recognition module is not running in lhand')
        hironxsys.HiroNxSystem.connect(self)

    def detect(self, sensor='rhandcam', timeout=1.5):
        def read_stable_result(hrecog):
            lastpos = zeros(3)
            start = time.time()
            lasttm = RTC.Time(sec=0, nsec=0)
            while time.time() - start < timeout:
                pose3d_stamped = hrecog.read()
                if pose3d_stamped == None:
                    continue
                tm = pose3d_stamped.tm
                a = array(pose3d_stamped.data)[8:].reshape(3,4)
                m = a[:,:3]
                v = a[:,3]

                print tm.sec + tm.nsec*1e-9, ' ', v[0], ' ', v[1]

                if tm.sec + tm.nsec*1e-9 > lasttm.sec + lasttm.nsec*1e-9:
                    pos = v
                    if linalg.norm(pos-lastpos) < 10:
                        return a
                    elif linalg.norm(pos) > 1:
                        lastpos = pos
                        lasttm = tm
                time.sleep(0.1)

            return None

        if sensor == 'rhandcam':
            hrecog = self.hrhandrecog
        else:
            hrecog = self.hlhandrecog

        return read_stable_result(hrecog)
