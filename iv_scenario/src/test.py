#!/usr/bin/env python
# -*- coding: utf-8 -*-

# set up environment for ROS/RTM
# RTM nameserver is read from rtc.conf file in the current directory

import sys
sys.path.insert(0, '/home/leus/prog/hironx/iv_plan_hironx/iv_plan/src')
from ivenv import *
ivpkgdir,nameserver = setup_ivenv()

import scene_objects
from hironx import *
from mplan_env import *
from csplan import *

import hironx_params
import hironxsys
rr = hironxsys.HiroNxSystem(nameserver, hironx_params.portdefs)

env = PlanningEnvironment()
env.load_scene(scene_objects.table_scene())

r = HiroNx(ivenv.ivpkgdir+'/iv_plan/externals/models/HIRONX_110822/')
env.insert_robot(r)
r.go_pos(-150, 0, 0)
r.prepare()

pl = CSPlanner(env)

import toplevel_extension
toplevel_extension.setup_toplevel_extension(r,env,rr,pl)
from toplevel_extension import *


def test1():
    js = 'rarm'
    f1 = FRAME(xyzabc=[200,120,800,0,-pi/2,0])
    traj1 = move_arm_plan(f1, joints=js)
    show_traj(traj1, joints=js)
    js = 'larm'
    f2 = FRAME(xyzabc=[250,-120,1000,0,-pi/2,0])
    traj2 = move_arm_plan(f2, joints=js)
    show_traj(traj2, joints=js)
    js = 'all'
    q0 = r.get_joint_angles(joints=js)
    r.prepare()
    q1 = r.get_joint_angles(joints=js)
    traj3 = pl.make_plan(q0, q1, joints=js)
    show_traj(traj3, joints=js)
