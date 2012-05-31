# -*- coding: utf-8 -*-

# set up environment for ROS/RTM
# RTM nameserver is read from rtc.conf file in the current directory

from ivenv import *
import scene_objects
from hironx import *
from mplan_env import *
from csplan import *

import hironx_params
import hironxsys

real_robot = False
if real_robot:
    rr = hironxsys.HiroNxSystem(hironx_params.portdefs)
else:
    rr = None

env = PlanningEnvironment()
env.load_scene(scene_objects.table_scene())

r = HiroNx(ivenv.ivpkgdir+'/iv_plan/externals/models/HIRONX_110822/', forcesensors=False)
env.insert_robot(r)
r.go_pos(-150, 0, 0)
r.prepare()

pl = CSPlanner(env)

import toplevel_extension
toplevel_extension.setup_toplevel_extension(r,env,rr,pl)
from toplevel_extension import *


def test1():
    traj = []
    r.reset_pose()
    jts = 'rarm'
    f1 = FRAME(xyzabc=[220,120,800,0,-pi/2,0])
    traj.append((move_arm_plan(f1, joints=jts), jts))
    jts = 'larm'
    f2 = FRAME(xyzabc=[250,-120,1000,0,-pi/2,0])
    traj.append((move_arm_plan(f2, joints=jts), jts))
    f3 = FRAME(xyzabc=[100,400,800,0,-pi/2,0])
    traj.append((move_arm_plan(f3, joints=jts), jts))
    jts = 'all'
    q0 = r.get_joint_angles(joints=jts)
    r.prepare()
    q1 = r.get_joint_angles(joints=jts)
    traj.append((pl.make_plan(q0, q1, joints=jts), jts))

    for seg,jts in traj:
        show_traj(seg, joints=jts)
        time.sleep(0.5)

    return traj
