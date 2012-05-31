# -*- coding: utf-8 -*-

from numpy import *

import optparse
import os.path
import sys
import threading
import time
import traceback
import rtctree.tree
import rtctree.utils

import OpenRTM_aist

from rtshell import comp_mgmt
from rtshell import modmgr
import rtshell.path
from rtshell import port_types
import rtprint_comp
#import rtshell

from set_env import *
#import RTC_grx
#import RTC


prompt = 'continue? (y/n): '

def start_reading_port(raw_paths, options, tree=None):
    event = threading.Event()

    mm = modmgr.ModuleMgr(verbose=options.verbose, paths=options.paths)
    mm.load_mods_and_poas(options.modules)
    if options.verbose:
        print >>sys.stderr, \
                'Pre-loaded modules: {0}'.format(mm.loaded_mod_names)
    if options.timeout == -1:
        max = options.max
        if options.verbose:
            print >>sys.stderr, 'Will run {0} times.'.format(max)
    else:
        max = -1
        if options.verbose:
            print >>sys.stderr, 'Will stop after {0}s'.format(options.timeout)

    targets = port_types.parse_targets(raw_paths)
    if not tree:
        paths = [t[0] for t in targets]
        tree = rtctree.tree.RTCTree(paths=paths, filter=paths)
    port_specs = port_types.make_port_specs(targets, mm, tree)
    port_types.require_all_input(port_specs)
    if options.verbose:
        print >>sys.stderr, \
                'Port specifications: {0}'.format([str(p) for p in port_specs])

    comp_name, mgr = comp_mgmt.make_comp('iv_scenario', tree,
            rtprint_comp.Reader, port_specs, event=event, rate=options.rate,
            max=max)
    if options.verbose:
        print >>sys.stderr, 'Created component {0}'.format(comp_name)
    comp = comp_mgmt.find_comp_in_mgr(comp_name, mgr)
    comp_mgmt.connect(comp, port_specs, tree)
    comp_mgmt.activate(comp)

    return comp, mgr, tree
    try:
        if options.timeout != -1:
            event.wait(options.timeout)
            comp_mgmt.disconnect(comp)
            comp_mgmt.deactivate(comp)
        elif options.max > -1:
            event.wait()
            result = comp.get()
            comp_mgmt.disconnect(comp)
            comp_mgmt.deactivate(comp)
        else:
            while True:
                raw_input()
            # The manager will catch the Ctrl-C and shut down itself, so don't
            # disconnect/deactivate the component
    except KeyboardInterrupt:
        pass
    except EOFError:
        pass

    tree.give_away_orb()
    del tree
    comp_mgmt.shutdown(mgr)
    return result

#ports = ['Recognition0.rtc:RecognitionResultOut']
#ports = ['RobotHardware0.rtc:jointStt', 'Flip0.rtc:boxPose']
ports = [nameserver+'/VisionPC.host_cxt/AppRecog0.rtc:boxPose',
         nameserver+'/lupus.host_cxt/AppRecog0.rtc:boxPose']
# ports = [nameserver+'/VisionPC.host_cxt/AppRecog0.rtc:boxPose']

# main(argv=[pt])
# options = {'paths': [], 'verbose': False, 'max': -1, 'modules': [], 'rate': 100.0, 'timeout': -1}

options = optparse.Values()
options.paths = []
options.verbose = False
options.max = -1
options.modules = ['RTC_grx']
options.rate = 10.0
options.timeout = -1
tree = None

comp = None
mgr = None
tree = None

def rtc_connect():
    global comp, mgr, tree
    comp,mgr,tree= start_reading_port([rtshell.path.cmd_path_to_full_path(p) for p in ports], options, tree)

def rtc_get(port):
    lastpos = zeros(3)
    lasttm = RTC.Time(sec=0, nsec=0)
    while True:
        pose3d_stamped = comp.get()[port]
        tm = pose3d_stamped.tm
        pose3d = pose3d_stamped.data

        if tm.sec > lasttm.sec or tm.nsec > lasttm.nsec:
            pos = array([pose3d.position.x, pose3d.position.y, pose3d.position.z])
            if linalg.norm(pos-lastpos) < 10:
                break
            elif linalg.norm(pos) > 1:
                lastpos = pos
                lasttm = tm

    return [pose3d.position.x, pose3d.position.y, pose3d.position.z,
            pose3d.orientation.r, pose3d.orientation.p, pose3d.orientation.y]

