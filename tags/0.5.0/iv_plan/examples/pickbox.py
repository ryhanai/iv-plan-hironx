# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *

def pick():
    r.set_joint_angles(r.poses['prepare_right'])
    r.grasp(width=100)
    b = env.get_object('box0')
    if b == None:
        warn('box0 not found')
        return
    f = b.where()
    afrms,gfrms = pl.grasp_plan(f)
    asol = r.ik(afrms, use_waist=True)[0]
    r.set_joint_angle(0, asol[0])
    r.set_arm_joint_angles(asol[1])
    gsol = r.ik(gfrms, use_waist=True)[0]
    r.set_joint_angle(0, gsol[0])
    r.set_arm_joint_angles(gsol[1])
    r.grasp(width=b.vbody.size[1]-2)
    affix_obj(b, hand='right')
    
def place():
    f0 = r.fk()
    f = FRAME(f0)
    f.vec[2] += 100
    th,avec = r.ik(f, use_waist=True)[0]
    r.set_joint_angle(0, th)
    r.set_arm_joint_angles(avec)
    sync(duration=2.0)
    
    f = FRAME(f0)
    f.vec[1] -= 100
    th,avec = r.ik(f, use_waist=True)[0]
    r.set_joint_angle(0, th)
    r.set_arm_joint_angles(avec)
    r.grasp(width=100)
    unfix_obj(env.get_object('box0'))
    sync(duration=2.0)
    
    r.reset_pose()

def affix_obj(obj, hand='right'):
    jname = 'RARM_JOINT5' if hand == 'right' else 'LARM_JOINT5'
    hj = r.get_joint(jname)
    reltf = (-hj.where())*obj.where()
    obj.unfix()
    obj.affix(hj, reltf)

def unfix_obj(obj):
    wfrm = obj.where()
    obj.unfix()
    obj.affix(env.get_world(), wfrm)

colored_print('1: putbox()', 'blue')
colored_print('2: pick()', 'blue')
colored_print('3: place()', 'blue')
