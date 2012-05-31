# -*- coding: utf-8 -*-
# Estimation of link=>sensor transforms needs ROS

from ivenv import *
from demo_common import *
import time
from tfpy import *
from scipy.optimize import leastsq

# q = [x,y,z,r,p,y,xp,yp,zp,nx,ny,nz]

neck_angles2 = [
    [0.32, 0.88], [0, 0.8], [-0.32, 0.88],
    [-0.32, 1.0], [0, 0.95], [0.32, 1.0], [0, 1.15]
    ]

r.prepare()

# def one_shot():
#     Tsen_tgt = rr.detect(camera='kinect_rgb')
#     r.set_joint_angles(rr.get_joint_angles())
#     Twld_hd = r.get_link('HEAD_JOINT1_Link').where()
#     show_frame(Twld_hd * r.Thd_kinectrgb * Tsen_tgt)
#     return r.get_joint_angles(), Tsen_tgt

def get_plane_param(f):
    n = VECTOR(vec=array(f.mat)[:,2].tolist())
    p = f.vec
    return n,p

def get_ground_truth():
    ft = env.get_object('table top').where()
    nw,pw = get_plane_param(ft)
    xyzabc = r.Thd_kinectdepth.xyzabc()
    v = xyzabc + pw + nw # state vector
    return v

def one_shot():
    fh = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectdepth
    ft = env.get_object('table top').where()
    fhh = (-fh)*ft
    nc,pc = get_plane_param(fhh)
    qf = [r.get_joint_angles(), pc, nc]
    return qf

def record_data(waitTime=2.0):
    qfs = []
    for y,p in neck_angles2:
        r.set_joint_angle(1,y)
        r.set_joint_angle(2,p)
        # sync()
        # time.sleep(waitTime)
        qfs.append(one_shot())
    return qfs

def error_fun1(Tx, qi, nci, pci, nw, pw):
    r.set_joint_angles(qi)
    Tqi = r.get_link('HEAD_JOINT1_Link').where()
    ndiff = 1 - dot((Tqi*Tx).mat*nci, nw)
    pdiff = abs(dot((Tqi*Tx*pci - pw), nw))
    print 'ndiff=', ndiff
    print 'pdiff=', pdiff
    return ndiff + pdiff

def error_fun(v, qfs):
    val = 0
    for q,pc,nc in qfs:
        Tx = FRAME(xyzabc=v[:6])
        pw = v[6:9]
        nw = v[9:12]
        val += error_fun1(Tx, q, nc, pc, nw, pw)
    return val

def compute_jacobi(v, res):
    for i in range(len(v)):
        dv = zeros(len(v))
        dv[i] = 0.1
        v1 = (v + dv).tolist()
        print i, error_fun(v1, res) - error_fun(v, res)

def calibrate(res, maxfev=2000, tf0=r.Thd_kinectdepth):
    freevars = res
    v0 = r.Thd_kinectdepth
    res = leastsq(error_fun, v0, args=freevars, maxfev=maxfev)
    print res
    
