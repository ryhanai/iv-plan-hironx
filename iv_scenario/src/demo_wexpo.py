# -*- coding: utf-8 -*-

from numpy import *
import sys
import time

from ivenv import *
import scene_objects
from hironx import *
from mplan_env import *
from csplan import *
import interface_wexpo

################################################################################
#
# Initialization Process:
#
# First instantiate 4 main instances, rtc-interface, planning environment,
# robot and planner.
# Then get useful top-level functions operating on these objects
# by calling setup_toplevel_extension().
#
################################################################################

real_robot = False
if real_robot:
    rr = interface_wexpo.MyHiroNxSystem(interface_wexpo.portdefs)
else:
    rr = None

env = PlanningEnvironment()
env.load_scene(scene_objects.table_scene())

r = HiroNx(ivenv.ivpkgdir+'/iv_plan/externals/models/HIRONX_110822/', forcesensors=True)
env.insert_robot(r)
r.go_pos(-150, 0, 0)
r.prepare()

pl = CSPlanner(env)

import toplevel_extension
toplevel_extension.setup_toplevel_extension(r,env,rr,pl)
from toplevel_extension import *

################################################################################
#
# Setup a scene for the demonstration in the simulator
#
################################################################################

init_frames = {}

tbltop = env.get_object('table top')
tblheight = tbltop.where().vec[2] + tbltop.vbody.size[2]/2

def reset_parts():
    for p in env.get_objects('^A'):
        p.locate(init_frames[p.name], world=True)

def reset_parts_randomly():
    def is_occupied(f, objs):
        for o in objs:
            if numpy.linalg.norm(f.vec - o.where().vec) < 70:
                return True
        return False

    objs = env.get_objects('^A')
    for p in objs:
        while True:
            x = 150*random.random()+140
            y = 300*random.random()-80
            z = tblheight + p.vbody.size[2]/2
            c = 2*pi * random.random()
            f = FRAME(xyzabc=[x,y,z,0,0,c])
            if not is_occupied(f, objs):
                p.locate(f)
                break

def init_palletizing_scene():
    global holeheight

    tbl = env.get_object('table')

    def put_on_table(objdef, name, xyzabc):
        obj = env.eval_sctree(objdef(name=name))
        env.insert_object(obj, FRAME(xyzabc=xyzabc), tbl)

    # put_on_table(scene_objects.pallete, 'pallete0', [-270,-250,tblheight-2,0,0,-pi/4])
    # copied from choreonoid settings
    # robot: [0, 159.900, 1059.800]
    # W: [296.980,-137.080,1041.640]
    # robot=>W: [296.980, -296.980, -18.160]

    put_on_table(scene_objects.pallete, 'pallete0', [-203.02,-296.98,tblheight-2,0,0,-pi/4])
    put_on_table(scene_objects.partsA, 'A0', [-180,-10,tblheight+15,0,0,pi/6])
    put_on_table(scene_objects.partsA, 'A1', [-130,100,tblheight+15,0,0,-pi/6])
    put_on_table(scene_objects.partsA, 'A2', [-160,190,tblheight+15,0,0,0])
    put_on_table(scene_objects.partsA, 'A3', [-100,-10,tblheight+15,0,0,pi/4])
    plt  = env.get_object('pallete0')

    def put_on_pallete(objdef, name, xyzabc):
        obj = env.eval_sctree(objdef(name=name))
        env.insert_object(obj, FRAME(xyzabc=xyzabc), plt)

    put_on_pallete(scene_objects.rect_pocket, 'P0', [40,40,20,0,0,0])
    put_on_pallete(scene_objects.rect_pocket, 'P1', [-40,40,20,0,0,0])
    put_on_pallete(scene_objects.rect_pocket, 'P2', [40,-40,20,0,0,0])
    put_on_pallete(scene_objects.rect_pocket, 'P3', [-40,-40,20,0,0,0])
    holeheight = 19 - 11 + env.get_object('A0').vbody.size[2]/2 # rubber thickness - hole height

    # remember the initial positions of the pieces in world frame
    for p in env.get_objects('^A'):
        init_frames[p.name] = FRAME(p.where())

    for o in env.get_objects('table top|pallete side|A|B'):
        r.add_collision_object(o)
    for o in env.get_objects('A'):
        env.add_collidable_object(o)


# fast version
# tms = {'preapproach1': 0.4,
#        'preapproach2': 1.0,
#        'pick': 0.65,
#        'transport': 0.8,
#        'place': 0.55,
#        'look_for': 0.5}

# normal speed
# tms = {'preapproach1': 1.2,
#        'preapproach2': 2.0,
#        'pick': 0.8,
#        'transport': 1.2,
#        'place': 0.8,
#        'pregrasp': 0.8,
#        'look_for': 0.7}

# slow version
tms = {'preapproach1': 1.5,
       'preapproach2': 2.5,
       'pick': 1.3,
       'transport': 1.7,
       'place': 1.3,
       'pregrasp': 0.7,
       'look_for': 0.9}


################################################################################
# Recognition
################################################################################

# Transform the frame of a target into the world frame
# after filtering some outliers.

def detect(timeout=1.5, zmin=tblheight, zmax=tblheight+250,
           constraint=None, sensor='rhandcam'):
    a = rr.detect(sensor=sensor, timeout=timeout)
    if a == None:
        return None
    Tcam_obj = FRAME(mat=a[:,:3].tolist(), vec=a[:,3].tolist())

    # outlier removal in the camera frame
    if Tcam_obj.vec[2] < 100.0:
        return None

    print 'cam=>obj: ', Tcam_obj

    # synchronize the model with the real robot pose
    q = rr.get_joint_angles()
    r.set_joint_angles(q)
    Twld_obj = r.get_sensor(sensor).where() * Tcam_obj

    # align the z-axis of the target to the world z-axis
    m = array(Twld_obj.mat)
    a = cross(m[0:3,2], [0,0,1])
    m2 = MATRIX(angle=linalg.norm(a), axis=VECTOR(vec=a.tolist()))
    Twld_obj.mat = m2*Twld_obj.mat

    print 'world=>obj: ', Twld_obj

    # outlier removal in the world frame
    if Twld_obj and Twld_obj.vec[2] > zmin and Twld_obj.vec[2] < zmax:
        if constraint != None:
            axis,thresh = constraint
            if abs(dot(array(Twld_obj.mat)[0:2,0], axis)) > thresh:
                return Twld_obj
        else:
            return Twld_obj


#
# observation planning (just go to fixed positions and try detection)
#

view_distance = 328
detectposs_dual = [[[230,-35],[230,150]], # x-y coords of each camera
                   [[280,-35],[280,150]],
                   [[330,-35],[330,150]]]

def go_scan_pose():
    rpos,lpos = detectposs_dual[0]
    jts = 'larm'
    fl = FRAME(xyzabc=[lpos[0], lpos[1], tblheight+view_distance, pi, 0, pi/2])*(-r.Tlh_cam)
    jts = 'rarm'
    fr = FRAME(xyzabc=[rpos[0], rpos[1], tblheight+view_distance, pi, 0, -pi/2])*(-r.Trh_cam)
    th = width2angle(100)
    move_lr(fl, fr, None, None, None, tms['preapproach2'])

def go_prepare_pose():
    q0 = r.get_joint_angles()
    r.prepare(width=100)
    fl = r.fk(arm='left')
    fr = r.fk(arm='right')
    r.set_joint_angles(q0)
    move_lr(fl, fr, 0.0, width2angle(100), width2angle(100), tms['preapproach2'])

def look_for():
    def already_detected(o, detected):
        for p in detected:
            if numpy.linalg.norm(p.vec - o.vec) < 40:
                return True
        return False

    go_scan_pose()

    detected = []
    for i,(rpos,lpos) in enumerate(detectposs_dual):
        if i != 0:
            jts = 'larm'
            fl = FRAME(xyzabc=[lpos[0], lpos[1], tblheight+view_distance, pi, 0, pi/2])*(-r.Tlh_cam)
            jts = 'rarm'
            fr = FRAME(xyzabc=[rpos[0], rpos[1], tblheight+view_distance, pi, 0, -pi/2])*(-r.Trh_cam)
            move_lr(fl, fr, None, None, None, tms['look_for'])
            time.sleep(1.5) # this is bad

        obj_fr = detect(sensor='rhandcam', timeout=1.5)
        obj_fl = detect(sensor='lhandcam', timeout=1.5)
        if obj_fr:
            if not already_detected(obj_fr, detected):
                detected.append(obj_fr)
        if obj_fl:
            if not already_detected(obj_fl, detected):
                detected.append(obj_fl)

        if len(detected) == 4:
            break

    for i,f in enumerate(detected):
        f.vec[2] = tblheight + 15
        env.get_object('A'+str(i)).locate(f, world=True)
    for i in range(len(detected),4): # not detected
        env.get_object('A'+str(i)).locate(FRAME(xyzabc=[500,-800,tblheight+16,0,0,0]))

    print '%d objects detected'%len(detected)
    if len(detected) < 4:
        raise RecognitionFailure()
    else:
        return True

def pocket_detection_pose(n):
    pocketpos = [(-30,45),(-110,45),
                 (-30,-45),(-110,-45)]
    x,y = pocketpos[n]
    plt = env.get_object('pallete0')
    fsoffset = 59
    z = tblheight+fsoffset+240 - plt.where().vec[2]
    return plt.where() * FRAME(xyzabc=[x,y,z,0,-pi/2,0])


################################################################################
# Motion Generation
################################################################################

def choose_objs(n=0):
    def aux(o1, o2):
        x = o1.where()
        y = o2.where()
        return cmp(x.vec[0]-x.vec[1], y.vec[0]-y.vec[1])

    objs = [o for o in env.get_objects('^A') if o.where().vec[1] > -100]

    if n == 0:
        objs.sort(cmp=aux)
        return objs[-1],objs[0]
    else:
        objs.sort(cmp=aux)
        o0 = objs[1]; o1 = objs[2]
        if o0.where().vec[1] < o1.where().vec[1]:
            return o0,o1
        else:
            return o1,o0

    return None

def try_IK(o, jts='rarm', long_side=False):
    s1, s2 = grasp_plan(o, long_side=long_side)
    try:
        afrm,gfrm,awidth,gwidth = s1
        rasol = r.ik(afrm, joints=jts)[0]
        rgsol = r.ik(gfrm, joints=jts)[0]
    except:
        try:
            afrm,gfrm,awidth,gwidth = s2
            rasol = r.ik(afrm, joints=jts)[0]
            rgsol = r.ik(gfrm, joints=jts)[0]
        except:
            warn('ik failed, '+jts)
            return None
    return rasol,rgsol,awidth,gwidth

def choose_and_pick():
    def aux(o1, o2):
        y1 = o1.where().vec[1]
        y2 = o2.where().vec[1]
        return cmp(y1, y2)

    objs = [o for o in env.get_objects('^A') if o.where().vec[1] > -100]
    objs.sort(aux)
    if len(objs) == 4:
        # choose 2
        rcandidates = objs[:2]
        lcandidates = objs[2:]
        lcandidates.reverse()
        for rc in rcandidates:
            sol = try_IK(rc, jts='rarm')
            if sol == None:
                continue
            rasol,rgsol,rawidth,rgwidth = sol
            for lc in lcandidates:
                print rc,
                print lc
                sol = try_IK(lc, jts='larm')
                if sol == None:
                    continue
                lasol,lgsol,lawidth,lgwidth = sol

                # check collision of the goal configuration
                q_start = r.get_joint_angles(joints='torso_arms')
                r.set_joint_angles(rasol, joints='rarm')
                r.set_joint_angles(lasol, joints='larm')
                q_goal = r.get_joint_angles(joints='torso_arms')
                if r.in_collision():
                    continue

                # check collision of the trajectories
                if pl.try_connect(q_start, q_goal, joints='torso_arms'):
                    warn('collision free')
                    sync(duration=tms['pick'])
                    r.set_joint_angles(rgsol, joints='rarm')
                    r.set_joint_angles(lgsol, joints='larm')
                    sync(duration=tms['pregrasp'])

                    # for w in [75,60,50,44,39,34]:
                    #     r.grasp(w, hand='right')
                    #     r.grasp(w, hand='left')
                    #     sync(duration=0.1)

                    r.grasp2(width2angle(38), joints='rhand')
                    r.grasp2(width2angle(38), joints='lhand')
                    sync(duration=0.5)

                    grab(hand='right')
                    grab(hand='left')

                    r.set_joint_angles(rasol, joints='rarm')
                    r.set_joint_angles(lasol, joints='larm')
                    sync(duration=tms['pregrasp'])
                    return True
                else:
                    warn('detect arm collision')

        warn('failed to pick two objects')
        return False

    elif len(objs) == 2:
        rc = objs[0]
        lc = objs[1]

        sol = try_IK(rc, jts='rarm')
        if sol == None:
            return False
        rasol,rgsol,rawidth,rgwidth = sol
        sol = try_IK(lc, jts='larm', long_side=True)
        if sol == None:
            return False
        lasol,lgsol,lawidth,lgwidth = sol

        # check collision of the goal configuration
        q_start = r.get_joint_angles(joints='torso_arms')
        r.set_joint_angles(rasol, joints='rarm')
        r.set_joint_angles(lasol, joints='larm')
        q_goal = r.get_joint_angles(joints='torso_arms')
        if r.in_collision():
            return False

        # check collision of the trajectories
        if pl.try_connect(q_start, q_goal, joints='torso_arms'):
            warn('collision free')
            sync(duration=tms['pick'])
            r.set_joint_angles(rgsol, joints='rarm')
            r.set_joint_angles(lgsol, joints='larm')
            sync(duration=tms['pregrasp'])

            # for rw,lw in zip([75,60,50,44,39,34],[75,65,55,50,46,43]):
            #     r.grasp(rw, hand='right')
            #     r.grasp(lw, hand='left')
            #     sync(duration=0.1)

            r.grasp2(width2angle(38), joints='rhand')
            r.grasp2(width2angle(48), joints='lhand')
            sync(duration=0.5)

            grab(hand='right')
            grab(hand='left')

            r.set_joint_angles(rasol, joints='rarm')
            r.set_joint_angles(lasol, joints='larm')
            sync(duration=tms['pregrasp'])
            return True
        else:
            warn('detect arm collision')

        warn('failed to pick two objects')
        return False

    else:
        warn('failed to pick two objects')
        return False


def grasp_plan(o, long_side=False, hand='right', inner_offset=3):
    appvec_length = 40
    objfrm = o.where()

    if long_side:
        objfrm = objfrm * FRAME(xyzabc=[0,0,0,0,0,pi/2])
        gwidth = o.vbody.size[0] - inner_offset
    else:
        gwidth = o.vbody.size[1] - inner_offset

    if hand == 'right':
        Twrist_ef = r.Trwrist_ef
    else:
        Twrist_ef = r.Tlwrist_ef

    awidth = gwidth + 35
    aangle = width2angle(awidth)
    gangle = width2angle(gwidth)

    gfrm = objfrm*(-Twrist_ef)
    afrm = FRAME(gfrm)
    afrm2 = objfrm*FRAME(xyzabc=[0,0,0,0,0,pi])*(-Twrist_ef)
    gfrm2 = objfrm*FRAME(xyzabc=[0,0,0,0,0,pi])*(-Twrist_ef)
    afrm.vec[2] += appvec_length
    afrm2.vec[2] += appvec_length
    return (afrm,gfrm,aangle,gangle),(afrm2,gfrm2,aangle,gangle)

def place_plan(p, hand='right'):
    plcvec_length = 60
    plcfrm = p.where()

    if hand == 'right':
	Twrist_ef = r.Trwrist_ef
    else:
        Twrist_ef = r.Tlwrist_ef

    gfrm = plcfrm*(-Twrist_ef)
    gfrm.vec[2] += holeheight
    afrm = FRAME(gfrm)
    afrm.vec[2] += plcvec_length

    gfrm2 = plcfrm*FRAME(xyzabc=[0,0,0,0,0,pi])*(-Twrist_ef)
    gfrm2.vec[2] += holeheight
    afrm2 = FRAME(gfrm2)
    afrm2.vec[2] += plcvec_length

    rwidth = 80
    rangle = width2angle(rwidth)
    return (afrm,gfrm,rangle),(afrm2,gfrm2,rangle)


rwp = FRAME(xyzabc=[200,-110,1049,0,-pi/2,0])
lwp = FRAME(xyzabc=[240,90,1049,0,-pi/2,0])


# choose_and_pick := try: move_lr(candidates_for_lr()) repeat


# TODO:
# collision checking
# move operations should throw exception (when IKFailure etc.)
# choose_and_pick() throws exception (IK failure or planning failure)

# Motion primitive definitions

def put_with_both_hands(lplace, rplace, duration):
    P0 = env.get_object(lplace)
    P3 = env.get_object(rplace)
    sl1, sl2 = place_plan(P0, hand='left')
    sr1, sr2 = place_plan(P3, hand='right')
    move_lr2([sl1, sl2], [sr1, sr2], None, duration, False)

def put_with_a_hand(place, duration, hand='right'):
    P = env.get_object(place)
    s1, s2 = place_plan(P, hand=hand)
    move2([s1, s1], None, duration, False)

def pass_left_to_right(torsoangle=-0.3, T=FRAME(xyzabc=[240, -10, 1050, -pi/2, 0, 0])):
    Tef_left = T*FRAME(xyzabc=[0,0,0,0,0,pi/6])
    Tef_right = Tef_left*FRAME(xyzabc=[0,0,0,pi,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])
    Tef_right1 = Tef_right*FRAME(xyzabc=[0,0,50,0,0,0])
    move_lr(Tef_left*(-r.Tlwrist_ef), Tef_right1*(-r.Trwrist_ef), torsoangle, None, width2angle(100), tms['pick'])
    move(Tef_right*(-r.Trwrist_ef), torsoangle, width2angle(38), tms['pick'], hand='right')
    release(hand='left')
    move(None, torsoangle, width2angle(80), 0.5, hand='left')
    grab(hand='right')
    move(Tef_right1*(-r.Trwrist_ef), torsoangle, None, tms['pick'], hand='right')


def demo(recognition=True):
    # scan the table and recognize objects
    if recognition:
        look_for()
    else:
        go_scan_pose()

    choose_and_pick() # pick_with_both_hands (if possible)
    move_lr(lwp, rwp, -0.3, None, None, tms['transport'])
    move_lr(pocket_detection_pose(0), pocket_detection_pose(3), -0.6, None, None, tms['transport'])
    # Here, recognize pockets on the pallet if necessary.
    # and adjust the locations of the pockets.
    put_with_both_hands('P0', 'P3', tms['place'])

    move_lr(lwp, rwp, 0.0, None, None, tms['transport'])
    choose_and_pick() # pick with a hand
    move_lr(lwp, rwp, -0.3, None, None, tms['transport'])
    move(pocket_detection_pose(2), None, None, tms['transport'], hand='right')
    # recognize the pocket
    put_with_a_hand('P2', tms['place'], hand='right')

    pass_left_to_right()
    move(pocket_detection_pose(1), -0.3, None, tms['transport'], hand='right')
    # recognize the pocket
    put_with_a_hand('P1', tms['place'], hand='right')
    go_prepare_pose()


init_palletizing_scene()

if rr:
    r.set_joint_angles(rr.get_joint_angles())
