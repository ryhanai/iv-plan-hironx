# -*- coding: utf-8 -*-

import sys
sys.path.append('../../iv_plan/src')

from set_env import *
from demo_common import *


def detect_kinect_rgb():
    '''チェスボードが貼られた箱の認識'''
    if rr:
        Tkinect_cb = rr.detect(camera='kinect_rgb')
        r.set_joint_angles(rr.get_joint_angles())
        print 'kinect_rgb->target:', Tkinect_cb
        Twld_cb = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectrgb*Tkinect_cb
        print 'world->target:', Twld_cb

        show_frame(Twld_cb)
        return Twld_cb
    else:
        warn('not yet supported')

def detect_kinect_point_center():
    if rr:
        Tkinect_cb = rr.detect(camera='kinect_point_center')
        r.set_joint_angles(rr.get_joint_angles())
        print 'kinect_point_center->target:', Tkinect_cb
        Twld_cb = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectdepth*Tkinect_cb
        print 'world->target:', Twld_cb

        show_frame(Twld_cb)
        return Twld_cb
    else:
        warn('not yet supported')

def detect_kinect_point_center_multi():
    if rr:
        Tkinect_cbs = rr.detect(camera='kinect_point_center')
        r.set_joint_angles(rr.get_joint_angles())
        Twld_kinectdepth = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectdepth
        res = []
        for Tkinect_cb in Tkinect_cbs:
            print 'kinect_point_center->target:', Tkinect_cb
            Twld_cb = Twld_kinectdepth*Tkinect_cb
            print 'world->target:', Twld_cb
            res.append(Twld_cb)

        cnt = 0
        for Twld_cb in res:
            show_frame(Twld_cb, name='frame'+str(cnt))
            cnt += 1
        return res
    else:
        warn('not yet supported')



#r.set_joint_angles(prepare_right2)
prepare_right2 = [0.24999878039410275,
                  -0.3999701506333021,
                  1.0999912242536529,
                  -0.242129102211388,
                  -0.55000849205344104,
                  -2.246649011302321,
                  -0.23170880440775701,
                  0.77617486969986105,
                  -0.064163644362093999,
                  0.009992009967667536,
                  0.0,
                  -1.7449976394364506,
                  -0.26498055477880189,
                  0.16399277276356095,
                  -0.055982450484027418,
                  0.78539814154175325,
                  -0.089011788368225098,
                  -0.78539814154175325,
                  0.090757124125957489,
                  -4.3711390063094768e-08,
                  0.013962633907794952,
                  0.0069812973353524654,
                  -0.015707964077591896]


def grasp_pet_with_kinect(f=None):
    if rr:
        f = detect_kinect_point_center()

    if f:
        fa = FRAME(xyzabc=[f.vec[0]-130,f.vec[1]-100,850,0,pi+0.1,-0.5])
        fg = FRAME(xyzabc=[f.vec[0]-55,f.vec[1]-50,850,0,pi+0.1,-0.5])
        w,av = r.ik(fa, use_waist=True)[0]
        r.set_joint_angle(0, w)
        r.set_arm_joint_angles(av)
        r.grasp(width=100)
        sync()
        w,av = r.ik(fg, use_waist=True)[0]
        r.set_joint_angle(0, w)
        r.set_arm_joint_angles(av)
        sync(duration=2.0)
        r.grasp(width=58)
        sync(duration=2.0)
        fp = r.fk()
        fp.vec[2] += 50
        r.set_arm_joint_angles(r.ik(fp)[0])
        sync(joints='rarm', duration=2.0)

def grasp_yellow_piece_with_kinect(f=None):
    if rr:
        f = detect_kinect_point_center()

    if f:
        fa = FRAME(xyzabc=[f.vec[0],f.vec[1],950,0,-pi/2,0])
        fg = FRAME(xyzabc=[f.vec[0],f.vec[1],900,0,-pi/2,0])
        r.set_arm_joint_angles(r.ik(fa)[0])
        r.grasp(width=100)
        sync()
        r.set_arm_joint_angles(r.ik(fg)[0])
        sync(duration=2.0)
        r.grasp(width=24)
        sync(duration=2.0)
        fp = r.fk()
        fp.vec[2] += 50
        r.set_arm_joint_angles(r.ik(fp)[0])
        sync(joints='rarm', duration=2.0)

def grasp_parts(f=None):
    if rr:
        f = detect_kinect_point_center()

    if f:
        fa = FRAME(xyzabc=[f.vec[0],f.vec[1],f.vec[2]+182.0,0,-pi/2,0])
        fg = FRAME(xyzabc=[f.vec[0],f.vec[1],f.vec[2]+132.0,0,-pi/2,0])
        r.set_arm_joint_angles(r.ik(fa)[0])
        r.grasp(width=100)
        sync(duration=2.0)
        r.set_arm_joint_angles(r.ik(fg)[0])
        sync(duration=1.5)
        if f.vec[2] > 730.0:
            r.grasp(width=24)
        else:
            r.grasp(width=34)
        sync(duration=1.5)
        fp = r.fk()
        fp.vec[2] += 50
        r.set_arm_joint_angles(r.ik(fp)[0])
        sync(joints='rarm', duration=1.5)

def move_arm(f, duration=2.0, arm='right', width=None, use_waist=True):
    if use_waist:
        w,avec = r.ik(f,arm=arm,use_waist=True)[0]
        r.set_joint_angle(0, w)
        r.set_arm_joint_angles(avec,arm=arm)
    else:
        r.set_arm_joint_angles(r.ik(f,arm=arm)[0], arm=arm)

    if width:
        r.grasp(width=width, hand=arm)
        if arm == 'right':
            hand_joints = '_rhand'
        else:
            hand_joints = '_lhand'
    else:
        hand_joints = ''
    if arm == 'right':
        arm_joints = 'rarm'
    else:
        arm_joints = 'larm'
    if use_waist:
        torso_joints = 'torso_'
    else:
        torso_joints = ''

    print torso_joints+arm_joints+hand_joints
    sync(duration=duration, joints=torso_joints+arm_joints+hand_joints)

def assemble_parts():
    while True:
        fs = detect_kinect_point_center_multi()
        # check the results
        if fs == None or len(fs) != 2:
            continue
        if fs[0].vec[2] < fs[1].vec[2]:
            fb = fs[0]
            fc = fs[1]
        else:
            fb = fs[1]
            fc = fs[0]
        if fb.vec[2] < 710 or fb.vec[2] > 750:
            continue
        if fc.vec[2] < 730 or fc.vec[2] > 780:
            continue

        fa = FRAME(xyzabc=[fb.vec[0],fb.vec[1],fb.vec[2]+175.0,0,-pi/2,0])
        fg = FRAME(xyzabc=[fb.vec[0],fb.vec[1],fb.vec[2]+125.0,0,-pi/2,0])
        move_arm(fa, duration=2.0, width=100)
        move_arm(fg, duration=1.5)
        r.grasp(width=35)
        sync(duration=1.5)

        f = r.fk()
        f.vec[2] += 50
        move_arm(f, duration=1.5)
        r.set_joint_angle(0, 0)
        sync(duration=1.5, joints='torso')
        f.vec[2] = 1000
        f = f*FRAME(xyzabc=[0,0,0,0,0,-pi/2])
        move_arm(f, use_waist=False)

        g = r.fk()
        g.vec[1] += 300
        g.vec[0] += 10
        g.vec[2] += 5
        g = g*FRAME(xyzabc=[0,0,0,0,0,pi/2])
        g = g*FRAME(xyzabc=[0,0,0,0,0,pi/2])
        g = g*FRAME(xyzabc=[0,0,0,pi/2,0,0])
        move_arm(g, duration=2.5, arm='left', use_waist=False)
        g.vec[1] -= 100
        move_arm(g, duration=2.5, arm='left', use_waist=False)
        r.grasp(width=45, hand='left')
        sync(joints='lhand', duration=1.5)
        r.grasp(width=100)
        sync(joints='rhand', duration=1.5)

        f = r.fk()
        f.vec[1] -= 100
        move_arm(f, use_waist=False)

        fa = FRAME(xyzabc=[fc.vec[0],fc.vec[1],fc.vec[2]+175.0,0,-pi/2,0])
        fg = FRAME(xyzabc=[fc.vec[0],fc.vec[1],fc.vec[2]+125.0,0,-pi/2,0])
        move_arm(fa, duration=2.0, width=100)
        move_arm(fg, duration=1.5)
        r.grasp(width=22)
        sync(duration=1.5, joints='rhand')

        f = r.fk()
        f.vec[2] += 50
        move_arm(f, duration=1.5)
        r.set_joint_angle(0, 0)
        sync(duration=1.5, joints='torso')
        f.vec[2] = 1000
        f = f*FRAME(xyzabc=[0,0,0,0,0,-pi/2])
        move_arm(f, use_waist=False)

        g = r.fk(arm='left')
        g.vec[1]-=200
        g=g*FRAME(xyzabc=[0,0,0,-pi/2,0,0])
        g=g*FRAME(xyzabc=[0,0,0,0,0,pi/2])
        g=g*FRAME(xyzabc=[0,0,0,0,0,pi/2])
        g.vec[0] -= 5
        g.vec[2] -= 6
        move_arm(g, use_waist=False)

        break



def hoge():
    while True:
        fs = detect_kinect_point_center_multi()
        # check the results
        if fs == None or len(fs) != 2:
            continue
        if fs[0].vec[2] < fs[1].vec[2]:
            fb = fs[0]
            fc = fs[1]
        else:
            fb = fs[1]
            fc = fs[0]
        if fb.vec[2] < 710 or fb.vec[2] > 750:
            continue
        if fc.vec[2] < 730 or fc.vec[2] > 780:
            continue

        fa = FRAME(xyzabc=[fc.vec[0],fc.vec[1],fc.vec[2]+175.0,0,-pi/2,0])
        fg = FRAME(xyzabc=[fc.vec[0],fc.vec[1],fc.vec[2]+125.0,0,-pi/2,0])
        move_arm(fa, duration=2.0, width=100)
        move_arm(fg, duration=1.5)
        r.grasp(width=22)
        sync(duration=1.5, joints='rhand')
        f = r.fk()
        f.vec[2] += 80
        move_arm(f, duration=1.5)

        fa = FRAME(xyzabc=[fb.vec[0]+7,fb.vec[1]+7,fb.vec[2]+215.0,0,-pi/2,0])
        fg = FRAME(xyzabc=[fb.vec[0],fb.vec[1],fb.vec[2]+185.0,0,-pi/2,0])
        move_arm(fa, duration=2.0)

        f = r.fk()
        f.vec[2] -= 30
        move_arm(f)

        r.grasp(width=23.5)
        sync(duration=1.5, joints='rhand')
        f.vec[0] -= 2
        move_arm(f,duration=1.0)
        f.vec[1] -= 2
        move_arm(f,duration=1.0)
        f.vec[0] -= 2
        move_arm(f,duration=1.0)
        f.vec[1] -= 2
        move_arm(f,duration=1.0)
        f.vec[0] -= 2
        move_arm(f,duration=1.0)
        f.vec[1] -= 2
        move_arm(f,duration=1.0)
        f.vec[0] -= 2
        move_arm(f,duration=1.0)
        f.vec[1] -= 2
        move_arm(f,duration=1.0)
        r.grasp(width=30)
        sync(duration=2.0, joints='rhand')

        # move_arm(fg, duration=1.5)
        # r.grasp(width=35)
        # sync(duration=1.5)

        # f = r.fk()
        # f.vec[2] += 50
        # move_arm(f, duration=1.5)
        # r.set_joint_angle(0, 0)
        # sync(duration=1.5, joints='torso')
        # f.vec[2] = 1000
        # f = f*FRAME(xyzabc=[0,0,0,0,0,-pi/2])
        # move_arm(f, use_waist=False)

        # g = r.fk()
        # g.vec[1] += 300
        # g.vec[0] += 10
        # g.vec[2] += 5
        # g = g*FRAME(xyzabc=[0,0,0,0,0,pi/2])
        # g = g*FRAME(xyzabc=[0,0,0,0,0,pi/2])
        # g = g*FRAME(xyzabc=[0,0,0,pi/2,0,0])
        # move_arm(g, duration=2.5, arm='left', use_waist=False)
        # g.vec[1] -= 100
        # move_arm(g, duration=2.5, arm='left', use_waist=False)
        # r.grasp(width=45, hand='left')
        # sync(joints='lhand', duration=1.5)
        # r.grasp(width=100)
        # sync(joints='rhand', duration=1.5)

        # f = r.fk()
        # f.vec[1] -= 100
        # move_arm(f, use_waist=False)

        break


