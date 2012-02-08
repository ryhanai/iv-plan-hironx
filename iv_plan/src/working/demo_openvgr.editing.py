# -*- coding: utf-8 -*-

#import roslib; roslib.load_manifest('iv_plan')

from numpy import *
import sys
import time

from setup_rtshell import *


# quick version
# tms = {'preapproach1': 0.4,
#        'preapproach2': 1.0,
#        'pick': 0.65,
#        'transport': 0.6,
#        'place': 0.55}

# slow version
tms = {'preapproach1': 1.5,
       'preapproach2': 2.5,
       'pick': 1.5,
       'transport': 1.5,
       'place': 1.5}

detectposs = [(160,-50),(230,-50),
              (150, 10),(220, 10)]

# pocketposs = [(200,-300),(120,-300),
#               (200,-380),(120,-380)]
pocketposs = [(180,-240),(100,-240),
                   (180,-330),(100,-330)]


def preapproach(n = 0):
    print 'PRE:', n
    f = r.fk()
    f.vec[2] += 60
    sol = r.ik(f)[0]
    r.set_arm_joint_angles(sol)
    sync(joints='rarm', duration=tms['preapproach1'])

    r.prepare(width=80)
    x,y = detectposs[n]
    f = FRAME(xyzabc=[x, y, 1050,0,-pi/2,0])
    r.set_arm_joint_angles(r.ik(f)[0])
    sync(duration=tms['preapproach2'])

# def get_joint_angles():
#     return reduce(operator.__add__, comp.get()[0].qState)

def detect_pose3d(scl=1.0, hand='right'):
    if hand == 'right':
        port = 0
        parentlink = 'RARM_JOINT5_Link'
        Th_cam = r.Trh_cam
    else:
        port = 1
        parentlink = 'LARM_JOINT5_Link'
        Th_cam = r.Tlh_cam

    pose3d = rtc_get(port)
    if pose3d[2] < 100.0:
        return None

    x,y,z,ax,ay,az = pose3d
    axis =  array([ax,ay,az])
    angle = linalg.norm(axis)

    m = MATRIX(angle=angle, axis=VECTOR(vec=axis.tolist()))
    Tcam_obj = FRAME(mat=m, vec=VECTOR(x,y,z))

    print 'cam=>obj: ', Tcam_obj

    q = rr.get_joint_angles()
    r.set_joint_angles(q)
    Twld_cam = r.get_link(parentlink).where()*Th_cam

    Twld_obj = Twld_cam * Tcam_obj

    m = array(Twld_obj.mat)
    a = cross(m[0:3,2], [0,0,1])
    m2 = MATRIX(angle=linalg.norm(a), axis=VECTOR(vec=a.tolist()))
    Twld_obj.mat = m2*Twld_obj.mat

    return Twld_obj

def pick(f, hand='right', h = 722):
    if hand == 'right':
        Twrist_ef = r.Trwrist_ef
        jts = 'rarm'
        hjts = 'rhand'
        h = 720
    else:
        Twrist_ef = r.Tlwrist_ef
        jts = 'larm'
        hjts = 'lhand'

    f.vec[2] = h

    f2 = f * (-Twrist_ef)
    sols = r.ik(f2, joints=jts)
    if sols == []:
        f2 = f * FRAME(xyzabc=[0,0,0,0,0,pi]) * (-Twrist_ef)
        sols = r.ik(f2, joints=jts)
    r.set_joint_angles(sols[0], joints=jts)
    sync(joints=jts, duration=tms['pick'])
    for w in [80,60,50,44,39,34]:
        r.grasp(w, hand=hand)
        sync(duration=0.2, joints=hjts)

def transport(n = 0):
    f = r.fk()
    f.vec[2] += 150
    sol = r.ik(f)[0]
    r.set_arm_joint_angles(sol)
    sync(joints='rarm', duration=tms['transport'])
    x,y = pocketposs[n]
    f = FRAME(xyzabc=[x, y, 975,0,-pi/2,0])
    r.set_arm_joint_angles(r.ik(f)[0])
    sync(joints='rarm', duration=tms['transport'])

def place(f, h = 738, dosync=True):
    f.vec[2] = h
    f2 = f * (-r.Twrist_ef)
    sols = r.ik(f2)
    if sols == []:
        f2 = f * FRAME(xyzabc=[0,0,0,0,0,pi]) * (-r.Twrist_ef)
        sols = r.ik(f2)
    r.set_arm_joint_angles(sols[0])
    if dosync:
        sync(joints='rarm', duration=tms['place'])
        for w in [38,40,46,80]:
            r.grasp(w)
            sync(duration=0.2)

def detect(zmin=710, hand='right'):
    while True:
        f = detect_pose3d(hand=hand)
        if f and f.vec[2] > zmin and f.vec[2] < 745:
            return f

def pick_and_place(n=1):
    preapproach(0)
    for i in range(n):
        f = detect()
        pick(f)
        transport(i)
        f = detect(zmin=680)
        place(f)
        if i == n-1:
            preapproach(0)
        else:
            preapproach(i+1)

pocketposs_dual = [(180,-240),(100,-240),
                   (180,-330),(100,-330)]

detectposs_dual = [(160,-30),(160,150)]

def preapproach_dual():
    r.prepare(width=80)

    jts = 'rarm'
    x,y = detectposs_dual[0]
    f = FRAME(xyzabc=[x, y, 1025,0,-pi/2,0])
    r.set_joint_angles(r.ik(f, jts)[0], joints=jts)
    
    jts = 'larm'
    x,y = detectposs_dual[1]
    f = FRAME(xyzabc=[x, y, 1025,0,-pi/2,0])
    r.set_joint_angles(r.ik(f, jts)[0], joints=jts)

    sync(duration=tms['preapproach2'])
    

def dual_arm_pick_and_place(oname00='A0', oname01='A2',
                            pname00='P3', pname01='P0'):

    preapproach_dual()

    ## pick ##
    rofrm = detect(hand='right')
    lofrm = detect(hand='left')
    rofrm.vec[2] = 720
    lofrm.vec[2] = 722
    # adjust the positions of target objects
    A0 = env.get_object('A0')
    A0.locate(rofrm, world=True)
    A2 = env.get_object('A2')
    A2.locate(lofrm, world=True)

    q0 = r.get_joint_angles(joints='torso_arms')

    jts = 'rarm'
    parts = env.get_object(name=oname00)
    afrm,gfrm,handwidth = graspplan(objtype(parts), parts.where())

    try:
        afrm_rarm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_rarm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        afrm,gfrm,handwidth = request_next(afrm,gfrm,handwidth)

    try:
        afrm_rarm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_rarm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        warn('ik solution not found: %s'%jts)
        return

    jts = 'larm'
    parts = env.get_object(name=oname01)
    afrm,gfrm,handwidth = graspplan(objtype(parts), parts.where())

    try:
        afrm_larm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_larm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        afrm,gfrm,handwidth = request_next(afrm,gfrm,handwidth)

    try:
        afrm_larm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_larm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        warn('ik solution not found: %s'%jts)
        return

    r.set_joint_angles(afrm_rarm_sol, joints='rarm')
    r.set_joint_angles(afrm_larm_sol, joints='larm')
    q1 = r.get_joint_angles(joints='torso_arms')
    traj = pl.make_plan(q0, q1, joints='torso_arms')
    exec_traj(traj, joints='torso_arms')

    r.set_joint_angles(gfrm_rarm_sol, joints='rarm')
    r.set_joint_angles(gfrm_larm_sol, joints='larm')
    sync(duration=1.0)
    r.grasp(width=34, hand='right')
    r.grasp(width=34, hand='left')
    sync(duration=1.0)
    grab(hand='right')
    grab(hand='left')
    r.set_joint_angles(afrm_rarm_sol, joints='rarm')
    r.set_joint_angles(afrm_larm_sol, joints='larm')
    sync(duration=1.0)

    ## transport ##
    q0 = r.get_joint_angles(joints='torso_arms')
    r.set_joint_angle(0, -0.6)

    jts = 'rarm'
    x,y = pocketposs_dual[3] # right
    f = FRAME(xyzabc=[x, y, 975,0,-pi/2,0])
    r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
    jts = 'larm'
    x,y = pocketposs_dual[0] # left
    f = FRAME(xyzabc=[x, y, 975,0,-pi/2,0])
    r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
    q1 = r.get_joint_angles(joints='torso_arms')
    traj = pl.make_plan(q0, q1, joints='torso_arms')
    exec_traj(traj, joints='torso_arms')

    ## place ##
    rpfrm = detect(hand='right', zmin=680)
    lpfrm = detect(hand='left', zmin=680)
    rpfrm.vec[2] = 724
    lpfrm.vec[2] = 726
    # adjust the positions of pockets
    P3 = env.get_object('P3')
    P3.locate(rpfrm, world=True)
    P0 = env.get_object('P0')
    P0.locate(lpfrm, world=True)

    q0 = r.get_joint_angles(joints='torso_arms')

    jts = 'rarm'
    plc = env.get_object(name=pname00)
    afrm,gfrm = placeplan(objtype(parts), plc.where())

    try:
        afrm_rarm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_rarm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        afrm,gfrm,_ = request_next(afrm,gfrm,0)

    try:
        afrm_rarm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_rarm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        warn('ik solution not found: %s'%jts)
        return

    jts = 'larm'
    plc = env.get_object(name=pname01)
    afrm,gfrm = placeplan(objtype(parts), plc.where())

    try:
        afrm_larm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_larm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        afrm,gfrm,_ = request_next(afrm,gfrm,0)

    try:
        afrm_larm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_larm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        warn('ik solution not found: %s'%jts)
        return

    r.set_joint_angles(afrm_rarm_sol, joints='rarm')
    r.set_joint_angles(afrm_larm_sol, joints='larm')
    q1 = r.get_joint_angles(joints='torso_arms')
    traj = pl.make_plan(q0, q1, joints='torso_arms')
    exec_traj(traj, joints='torso_arms')

    r.set_joint_angles(gfrm_rarm_sol, joints='rarm')
    r.set_joint_angles(gfrm_larm_sol, joints='larm')
    sync(duration=0.5, joints='torso_arms')
    r.grasp(width=80, hand='right')
    r.grasp(width=80, hand='left')
    sync(duration=0.5)
    release(hand='right')
    release(hand='left')
    r.set_joint_angles(afrm_rarm_sol, joints='rarm')
    r.set_joint_angles(afrm_larm_sol, joints='larm')
    sync(duration=0.5)

    go_prepare_pose()


plt = env.get_object('pallete0')
plt.set_trans(FRAME(xyzabc=[-290,-270,700,0,0,0]))
A0 = env.get_object('A0')
f = A0.rel_trans
f.vec[1] += 40
A0.locate(f)
env.delete_object('A1')
env.delete_object('A3')
env.delete_object('B0')
env.delete_object('B1')



# def main():
#     try:
#         val = read_from_ports([path.cmd_path_to_full_path(pt)], options, tree)
#         val = val.data
#         f = array(val[8:20])
#         f.resize([3,4])
#         print f[0:3,3]
#         Twld_obj = convert(f)
#         show_frame(Twld_obj)
#         key = raw_input(prompt)
#         if key != 'y':
#             return f
#         pickup(Twld_obj)
#     except Exception, e:
#         print >> sys.stderr, '{0}: {1}'.format(os.path.basename(sys.argv[0]), e)
#         return 1
#     return 0


# f = array([[  9.98929562e-01,   0.00000000e+00,   4.62572077e-02,
#               -1.50504138e+01],
#            [  3.20064736e-02,  -7.21970315e-01,  -6.91183369e-01,
#               -3.07515927e+01],
#            [  3.33963308e-02,   6.91924030e-01,  -7.21197491e-01,
#               7.43907387e+02]])

# def convert(f):
#     Tcam_obj = FRAME(mat=f[0:3,0:3].tolist(),vec=f[0:3,3].tolist())
#     Twld_cam = FRAME(xyzabc=[795,-170,1330,pi/2,-pi/2,0])*FRAME(mat=MATRIX(a=51.8*pi/180.0))
#     Twld_obj = Twld_cam*Tcam_obj
#     print 'world->obj: ', Twld_obj
#     return Twld_obj

# def pickup(f):
#     fa = FRAME(xyzabc=[f.vec[0],f.vec[1],f.vec[2]+202.0,0,-pi/2,0])
#     fg = FRAME(xyzabc=[f.vec[0],f.vec[1],f.vec[2]+152.0,0,-pi/2,0])
#     w,avec = r.ik(fa, use_waist=True)[0]
#     r.set_joint_angle(0, w)
#     r.set_arm_joint_angles(avec)
#     r.grasp(width=100)
#     sync(duration=2.0)
#     key = raw_input(prompt)
#     if key != 'y':
#         return
#     w,avec = r.ik(fg, use_waist=True)[0]
#     r.set_joint_angle(0, w)
#     r.set_arm_joint_angles(avec)
#     sync(duration=1.5)
#     r.grasp(width=24)
#     sync(duration=1.5)
#     fp = r.fk()
#     fp.vec[2] += 50
#     w,avec = r.ik(fp, use_waist=True)[0]
#     r.set_joint_angle(0, w)
#     r.set_arm_joint_angles(avec)
#     sync(joints='rarm', duration=1.5)
