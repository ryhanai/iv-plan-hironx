# -*- coding: utf-8 -*-
# Estimation of link=>sensor transforms needs ROS

from set_env import *
from demo_common import *
import time
from tfpy import *
from scipy.optimize import leastsq

# hand camera calibration pose
def handcam_calib_pose():
    rf = FRAME(xyzabc=[200,-300,1100,0,-pi,0])
    jts = 'rarm'
    r.set_joint_angles(r.ik(rf, joints=jts)[0], joints=jts)
    lf = FRAME(xyzabc=[200,300,1100,0,-pi,0])
    jts = 'larm'
    r.set_joint_angles(r.ik(lf, joints=jts)[0], joints=jts)


neck_angles1 = [
    [0.35, 0.85], [0, 0.8], [-0.35, 0.85],
    [-0.4, 1.0], [0, 0.95], [0.4, 1.0], [0, 1.15]
    ]

neck_angles2 = [
    [0.32, 0.88], [0, 0.8], [-0.32, 0.88],
    [-0.32, 1.0], [0, 0.95], [0.32, 1.0], [0, 1.15]
    ]

height = 1100

# after r.prepare()
rhand_angles1 = [
    [270,-320,height,0,-pi/2,-pi/8],
    [270,-200,height,0,-pi/2,0],
    [270,-80,height,0,-pi/2,pi/8],
    [180,-200,height-20,0,-pi/2-pi/8,0],
    [380,-200,height-40,0,-pi/2+pi/8,0]
    ]

lhand_angles1 = [
    [270,320,height,0,-pi/2,pi/8],
    [270,200,height,0,-pi/2,0],
    [270,80,height,0,-pi/2,-pi/8],
    [180,200,height-20,0,-pi/2-pi/8,0],
    [380,200,height-40,0,-pi/2+pi/8,0]
    ]


def one_shot(sensor='kinect'):
    if sensor == 'kinect':
        Tsen_tgt = rr.detect(camera='kinect_rgb')
        r.set_joint_angles(rr.get_joint_angles())
        Twld_hd = r.get_link('HEAD_JOINT1_Link').where()
        show_frame(Twld_hd * r.Thd_kinectrgb * Tsen_tgt)
        return r.get_joint_angles(), Tsen_tgt
    elif sensor == 'rhand_cam':
        tf = rr.get_tf('/base_link', '/checkerboard')
        (trans, rot) = tf
        Tsen_tgt = FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                         vec=VECTOR(vec=(1000.0*array(trans)).tolist()))
        r.set_joint_angles(rr.get_joint_angles())
        Twld_rh = r.get_link('RARM_JOINT5_Link').where()
        show_frame(Twld_rh * r.Trh_cam * Tsen_tgt)
        return r.get_joint_angles(), Tsen_tgt
    elif sensor == 'lhand_cam':
        tf = rr.get_tf('/base_link', '/checkerboard')
        (trans, rot) = tf
        Tsen_tgt = FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                         vec=VECTOR(vec=(1000.0*array(trans)).tolist()))
        r.set_joint_angles(rr.get_joint_angles())
        Twld_lh = r.get_link('LARM_JOINT5_Link').where()
        show_frame(Twld_lh * r.Tlh_cam * Tsen_tgt)
        return r.get_joint_angles(), Tsen_tgt

def record_data(sensor='kinect', waitTime=2.0):
    res = []
    if sensor == 'kinect':
        for y,p in neck_angles2:
            r.set_joint_angle(1,y)
            r.set_joint_angle(2,p)
            sync()
            time.sleep(waitTime)
            res.append(one_shot(sensor))
    elif sensor == 'rhand_cam':
        r.prepare()
        sync()
        for v in rhand_angles1:
            f = FRAME(xyzabc=v)
            r.set_arm_joint_angles(r.ik(f)[0])
            sync()
            time.sleep(waitTime)
            res.append(one_shot(sensor))
    elif sensor == 'lhand_cam':
        r.prepare()
        sync()
        for v in lhand_angles1:
            jts = 'larm'
            f = FRAME(xyzabc=v)
            r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
            sync()
            time.sleep(waitTime)
            res.append(one_shot(sensor))
    return res

def play_data(res, link='HEAD_JOINT1_Link', tf=r.Thd_kinectrgb):
    for js, Tsen_tgt in res:
        r.set_joint_angles(js)
        Twld_tgt = r.get_link(link).where()*tf*Tsen_tgt
        show_frame(Twld_tgt)
        print Twld_tgt.vec
        time.sleep(1.0)

def distSO3(m1, m2):
    def quaternion(m):
        R = zeros((4,4))
        R[:3,:3] = m
        R[3,3] = 1
        return quaternion_from_matrix(R)

    q1 = quaternion(m1)
    q2 = quaternion(m2)
    return 1.0 - dot(q1, q2)

def error_fun_R(v, Ra, Rb):
    def e(v, Rai, Rbi):
        Rx = FRAME(xyzabc=[0,0,0]+v.tolist()).mat
        d =  distSO3(Rx*Rai, Rbi*Rx)
        # print v,d
        return d
    return [e(v,Rai,Rbi) for (Rai,Rbi) in zip(Ra,Rb)]

def error_fun_t(v, A, B, Rx):
    def e(v, Ai, Bi):
        Rbi = Bi.mat
        tx = v
        tai = Ai.vec
        tbi = Bi.vec
        error_v = dot(Rbi-identity(3), tx) - dot(Rx, tai) + tbi
        d = scipy.spatial.minkowski_distance(error_v, zeros(3), 2)
        # print v,d
        return d
    return [e(v,Ai,Bi) for (Ai,Bi) in zip(A,B)]

def error_fun_t2(v, qs, fs, Rx, h, link='HEAD_JOINT1_Link'):
    def e(v, q, f):
        r.set_joint_angles(q)
        Tx = FRAME(mat=Rx, vec=v.tolist())
        f2 = r.get_link(link).where()*Tx*f
        d = abs(f2.vec[2] - h)
        print v,d
        return d
    return [e(v,q,f) for (q,f) in zip(qs,fs)]

def calibrate(res, maxfev=2000, link='HEAD_JOINT1_Link', tf0=r.Thd_kinectrgb, height = 705.0):
    # height = 788 (on a pallete)

    qs,fs = unzip(res)
    B0 = []
    A0 = []

    for q in qs:
        r.set_joint_angles(q)
        B0.append(r.get_link(link).where())

    B = [(-x)*y for (x,y) in zip(B0[:-1],B0[1:])]
    A = [x*(-y) for (x,y) in zip(fs[:-1],fs[1:])]

    Rb = [b.mat for b in B]
    Ra = [a.mat for a in A]

    v0 = tf0.mat.abc()
    print 'maxfev = ', 2000

    print 'optimizing rotation ...'
    res = leastsq(error_fun_R, v0, args=(Ra,Rb), maxfev=maxfev)
    Rx = FRAME(xyzabc=[0,0,0]+res[0].tolist()).mat
    print 'rotation = ', Rx, ' rpy= ', res[0]

    v0 = tf0.vec
    print 'optimizing translation ...'
    #res = leastsq(error_fun_t, v0, args=(A,B,Rx), maxfev=maxfev)
    res = leastsq(error_fun_t2, v0, args=(qs,fs,Rx,height,link), maxfev=maxfev)
    tx = VECTOR(vec=res[0].tolist())
    print 'translation = ', tx

    return FRAME(mat=Rx, vec=tx)
