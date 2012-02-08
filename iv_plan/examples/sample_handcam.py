# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *
import hironx_motions

import RTC

def handcam_demo():
    '''右手ハンドカメラで箱を２つ見つけた後、box0をbox1の上に載せるデモ'''
    r.set_joint_angles(hironx_motions.stretch_right_pose)
    sync()

    objfrms = look_for_boxes()
    # 認識に成功したときは、
    # objfrms[0] が1番のマーカの箱, objfrms[1] が2番のマーカの箱の
    # 位置と姿勢が入っている

    if objfrms:
        # approach & grasp
        asols,gsols = pl.reaching_plan(objfrms[0], grasp_from_side=True, use_waist=True)
        (ayaw,a) = asols[0]
        (gyaw,g) = gsols[0]
        r.set_joint_angle(0, ayaw)
        r.set_arm_joint_angles(a)
        sync(duration=2.5)
        r.set_joint_angle(0, gyaw)
        r.set_arm_joint_angles(g)
        sync(duration=2.5)
        grasp(width=53) # 指先間隔を 53[mm] まで閉じる
        sync(duration=1.5)

        f1 = r.fk() # 現在の手先位置から一定量持ち上げる
        f1.vec[2] += 120
        wyaw,avec = r.ik(f1, use_waist=True)[0]
        r.set_joint_angle(0, wyaw)
        r.set_arm_joint_angles(avec)
        sync(joints='torso_rarm')

        # 積み重ねる場所へのアプローチ
        f2 = objfrms[1]
        f2.vec[2] += 70
        asols,gsols = pl.reaching_plan(f2, grasp_from_side=True, use_waist=True)
        (ayaw,a) = asols[0]
        (gyaw,g) = gsols[0]
        r.set_joint_angle(0, ayaw)
        r.set_arm_joint_angles(a)
        sync(duration=2.5, joints='torso_rarm')
        r.set_joint_angle(0, gyaw)
        r.set_arm_joint_angles(g)
        sync(duration=2.5, joints='torso_rarm')

        release(width=80)
        sync(duration=1.5)


def recog_pose():
    f = FRAME(xyzabc=[200,-300,930,0,-pi/2,0])
    q = r.ik(f)[0]
    r.set_arm_joint_angles(q)
    sync()

def detect_box():
    f = rr.read_pose3d()
    print 'cam => obj: ', f
    r.set_joint_angles(rr.get_joint_angles())
    Twld_cam = r.get_link('RARM_JOINT5_Link').where()*r.Trh_cam
    return Twld_cam * f


colored_print('1: putbox(name="box0", vaxis="y")', 'blue')
colored_print('2: putbox(name="box1", vaxis="y")', 'blue')
colored_print('3: handcam_demo()', 'blue')
