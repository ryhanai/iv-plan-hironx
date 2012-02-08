# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *

def demo_hold_basket():
    bk = env.eval_sctree(basket())
    env.insert_object(bk, FRAME(), env.get_object('table top'))
    bk.set_trans(FRAME(xyzabc=[-200,0,20,0,0,0]))
    s3 = env.get_object('basket side3')
    s4 = env.get_object('basket side4')

    s3frm = s3.where()*FRAME(xyzabc=[0,0,150,0,-pi/2,0])
    avec = r.ik(s3frm, arm='left')[0]
    r.set_arm_joint_angles(avec, arm='left')
    s4frm = s4.where()*FRAME(xyzabc=[0,0,150,0,-pi/2,0])
    avec = r.ik(s4frm)[0]
    r.set_arm_joint_angles(avec)
    sync()

    r.grasp(10)
    r.grasp(10, hand='left')
    handjnt = r.get_joint('RARM_JOINT5')
    reltf = (-handjnt.where())*bk.where()
    bk.unfix()
    bk.affix(handjnt, reltf)
    sync()

    grasp_height = r.fk().vec[2]
    
    s3frm = s3frm*FRAME(xyzabc=[100,0,0,0,0,0])
    s4frm = s4frm*FRAME(xyzabc=[100,0,0,0,0,0])
    r.set_arm_joint_angles(r.ik(s3frm, arm='left')[0], arm='left')
    r.set_arm_joint_angles(r.ik(s4frm)[0])
    sync()

    r.set_joint_angle(0, 0.5)
    sync()

    lfrm = r.fk(arm='left')
    lfrm.vec[2] = grasp_height
    r.set_arm_joint_angles(r.ik(lfrm, arm='left')[0], arm='left')
    rfrm = r.fk(arm='right')
    rfrm.vec[2] = grasp_height
    r.set_arm_joint_angles(r.ik(rfrm)[0])
    sync()
    
    r.grasp(40)
    r.grasp(40, hand='left')

    ttp = env.get_object('table top')
    reltf = (-ttp.where())*bk.where()
    bk.unfix()
    bk.affix(ttp, reltf)
