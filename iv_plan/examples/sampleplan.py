# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *

def demo_plan():
    def grasp_width(obj):
        if obj.vbody.__class__ == visual.cylinder:
            return obj.vbody.radius*2
        else:
            return obj.vbody.size[1]

    prepare_right()
    q_pre = r.get_arm_joint_angles()

    pltfrm = env.get_object('pallete').where()
    holefrm = pltfrm*FRAME(xyzabc=[65,41,0,0,0,0])

    # parameters
    appdist = {'partsA': 50, 'partsB': 100}
    grasp_zoffset = {'partsA': 0, 'partsB': 85}

    for tgtnm in ['partsA', 'partsB']:
        # grasp sequence
        pt = env.get_object(tgtnm)
        ptfrm = pt.where()
        ptfrm.vec[2] += grasp_zoffset[tgtnm]
        asols,gsols = pl.reaching_plan(ptfrm, use_waist=False)

        move_arm(asols[0])
        r.set_arm_joint_angles(gsols[0])
        grasp(width=grasp_width(pt), name=tgtnm)
        remove_hand_collision(tgtnm)
        grasped_height = ptfrm.vec[2]

        if tgtnm == 'partsB':
            remove_objects_collision('partsA', 'partsB')

        # place sequence
        tgtfrm = FRAME(holefrm)
        tgtfrm.vec[2] = grasped_height
        asols,gsols = pl.reaching_plan(tgtfrm, use_waist=False, approach_distance=appdist[tgtnm])
        move_arm(asols[0])
        r.set_arm_joint_angles(gsols[0])
        release(name = tgtnm)
        add_hand_collision(tgtnm)

        # go back to prepare pose
        move_arm(q_pre)

def setup_ac_scene():
    tbltop = env.get_object('table top')
    thickness = tbltop.vbody.size[2]
    name = 'pallete'
    plt = env.get_object('pallete')
    if not plt:
        newplt = env.eval_sctree(pallete(name))
        x,y,theta = -180, -370, 0
        reltf = FRAME(xyzabc=[x,y,(thickness+0)/2,0,0,pi/2])
        env.insert_object(newplt, reltf, tbltop)

    name = 'partsA'
    x,y,theta = -200, 100, 0
    if plt:
        pt = env.get_object(name)
        reltf = FRAME(xyzabc=[x,y,(thickness+pt.vbody.size[2])/2,
                              0,0,theta])
        pt.unfix()
        pt.affix(tbltop, reltf)
    else:
        pt = env.eval_sctree(partsA(name))
        reltf = FRAME(xyzabc=[x,y,(thickness+pt.vbody.size[2])/2,
                              0,0,theta])
        env.insert_object(pt, reltf, tbltop)

    name = 'partsB'
    x,y,theta = -200, -50, 0
    if plt:
        pt = env.get_object(name)
        reltf = FRAME(xyzabc=[x,y,thickness/2.0,0,0,theta])
        pt.unfix()
        pt.affix(tbltop, reltf)
    else:
        pt = env.eval_sctree(partsB(name))
        reltf = FRAME(xyzabc=[x,y,thickness/2.0,0,0,theta])
        env.insert_object(pt, reltf, tbltop)
