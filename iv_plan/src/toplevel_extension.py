# -*- coding: utf-8 -*-

import re
from viewer import *
from robot import *


# utils with no free variables
def objtype(obj):
    if re.match('^A.*', obj.name):
        return 1
    elif re.match('^B.*', obj.name):
        return 2
    elif re.match('^P.*', obj.name):
        return 3
    else:
        return 0


#
# Top-level utility functions,
# which is dependent on r, rr, pl, env.
#
def setup_toplevel_extension(r, env, rr=None, pl=None):
    global sync, sync_main, affix, unfix, move_arm_plan, move_arm
    global move_arm2, go_prepare_pose
    global move, move2, move_lr, move_lr2
    global show_frame, show_traj, show_tree, exec_traj
    global obj_in_hand, grab, release

    graspduration = 0.5

    def sync_main_(duration=4.0, joints='all', wait=True, waitkey=False):
        '''synchronize the real robot with the model in "duration" [sec]'''
        if rr:
            js = r.get_joint_angles()
            rr.send_goal(js, duration, wait=wait)
            return

            if joints == 'torso':
                rr.send_goal([js[0:3],[],[],[],[]], duration, wait=wait)
            elif joints == 'rarm':
                rr.send_goal([[],js[3:9],[],[],[]], duration, wait=wait)
            elif joints == 'larm':
                rr.send_goal([[],[],js[9:15],[],[]], duration, wait=wait)
            elif joints == 'rhand':
                rr.send_goal([[],[],[],js[15:19],[]], duration, wait=wait)
            elif joints == 'lhand':
                rr.send_goal([[],[],[],[],js[19:23]], duration, wait=wait)
            elif joints == 'torso_rarm':
                rr.send_goal([js[0:3],js[3:9],[],[],[]], duration, wait=wait)
            elif joints == 'torso_larm':
                rr.send_goal([js[0:3],[],js[9:15],[],[]], duration, wait=wait)
            elif joints == 'rarm_rhand':
                rr.send_goal([[],js[3:9],[],js[15:19],[]], duration, wait=wait)
            elif joints == 'torso_rarm_rhand':
                rr.send_goal([js[0:3],js[3:9],[],js[15:19],[]], duration, wait=wait)
            elif joints == 'larm_lhand':
                rr.send_goal([[],[],js[9:15],[],js[19:23]], duration, wait=wait)
            elif joints == 'torso_larm_lhand':
                rr.send_goal([js[0:3],[],js[9:15],[],js[19:23]], duration, wait=wait)
            elif joints == 'torso_arms':
                rr.send_goal([js[0:3],js[3:9],js[9:15],[],[]], duration, wait=wait)
            elif joints=='all':
                rr.send_goal([js[0:3],js[3:9],js[9:15],js[15:19],js[19:23]], duration, wait=wait)
            else:
                warn('unknown joints parameter: ' + joints)
        else:
            if waitkey:
                raw_input('type any key to continue')
            else:
                time.sleep(duration/4.0) # 4 times faster

    sync_main = sync_main_

    def sync_(duration=4.0, joints='all', wait=True, waitkey=False, goalthresh=0.2):
        sync_main(duration=duration, joints=joints, wait=wait, waitkey=waitkey)

    sync = sync_

# def sync(duration=4.0, joints='all', wait=True, waitkey=True, goalthresh=0.2):
#     while True:
#         sync_main(duration=duration, joints=joints, wait=wait, waitkey=waitkey)
#         qs = r.get_joint_angles(joints='all')
#         qr = rr.get_joint_angles()
#         if numpy.linalg.norm(array(qr) - array(qs)) < goalthresh:
#             return
#         else:
#             time.sleep(1)
    
    def affix_(obj, hand='right'):
        if hand == 'right':
            handjnt = r.get_joint('RARM_JOINT5')
        else:
            handjnt = r.get_joint('LARM_JOINT5')
        reltf = (-handjnt.where())*obj.where()
        obj.parent.children.remove(obj)
        obj.unfix()
        obj.affix(handjnt, reltf)
    affix = affix_

    def unfix_(obj, hand='right'):
        wldfrm = obj.where()
        obj.unfix()
        wld = env.get_world()
        wld.children.append(obj)
        obj.affix(wld, wldfrm)
    unfix = unfix_

    def move_arm_plan_(p1, joints='rarm'):
        '''move arm from current pose to p1'''
        q0 = r.get_joint_angles(joints=joints)
        q1 = r.ik(p1, joints=joints)[0]
        return pl.make_plan(q0, q1, joints=joints)
    move_arm_plan = move_arm_plan_

    def move_arm_(f, duration=2.0, joints='rarm', width=None, check_collision=False):
        if check_collision:
            traj = move_arm_plan(f, joints=joints)
            exec_traj(traj, joints=joints, duration=0.05)
        else:
            q = r.ik(f, joints=joints)[0]
            r.set_joint_angles(q, joints=joints)
            sync(duration=duration, joints=joints, waitkey=False)
        if width:
            rl,use_waist = parse_joints_flag(joints)
            r.grasp(width=width, hand=rl)
            sync(duration=0.5, joints=rl[0]+'hand', waitkey=False)
        return True
    move_arm = move_arm_

    def move_arm2_(afrm, gfrm, width, duration=2.0, joints='torso_rarm'):
        if r.ik(afrm, joints) == [] or r.ik(gfrm, joints) == []:
            return False
        else:
            move_arm(afrm, joints=joints, check_collision=True, duration=duration)
            move_arm(gfrm, width=width, joints=joints, check_collision=False, duration=0.5)
            return True
    move_arm2 = move_arm2_

    def go_prepare_pose_():
        jts = 'all'
        q0 = r.get_joint_angles(joints=jts)
        r.prepare()
        q1 = r.get_joint_angles(joints=jts)
        traj = pl.make_plan(q0, q1, joints=jts)
        if traj:
            exec_traj(traj, joints=jts)
            return True
        else:
            warn('error: go_prepare_pose()')
            return False
    go_prepare_pose = go_prepare_pose_

    def move_lr_(lframe, rframe, torsoangle, lhandangle, rhandangle, duration):
        q0 = r.get_joint_angles(joints='torso_arms')

        if torsoangle != None:
            r.set_joint_angle(0, torsoangle)
        if lframe != None:
            jts = 'larm'
            r.set_joint_angles(r.ik(lframe, joints=jts)[0], joints=jts)
        if rframe != None:
            jts = 'rarm'
            r.set_joint_angles(r.ik(rframe, joints=jts)[0], joints=jts)

        q1 = r.get_joint_angles(joints='torso_arms')
        jts = 'torso_arms'
        traj = pl.make_plan(q0, q1, joints=jts)
        exec_traj(traj, joints=jts, duration=duration)

        if lhandangle != None:
            r.grasp2(lhandangle, joints='lhand')
        if rhandangle != None:
            r.grasp2(rhandangle, joints='rhand')
        if lhandangle != None or rhandangle != None:
            sync(duration=graspduration)
    move_lr = move_lr_

    def move_lr2_(sls, srs, torsoangle, duration, grabFlag=True):
        def plan_and_execute(q0, lsol, rsol, duration):
            r.set_joint_angles(lasol, joints='larm')
            r.set_joint_angles(rasol, joints='rarm')
            q1 = r.get_joint_angles(joints='torso_arms')
            jts = 'torso_arms'
            traj = pl.make_plan(q0, q1, joints=jts)
            exec_traj(traj, joints=jts, duration=duration)

        def execute(lsol, rsol, duration):
            r.set_joint_angles(lsol, joints='larm')
            r.set_joint_angles(rsol, joints='rarm')
            sync(duration=duration)

        q0 = r.get_joint_angles(joints='torso_arms')

        if torsoangle != None:
            r.set_joint_angle(0, torsoangle)

        jts = 'larm'
        for sl in sls:
            try:
                afrm,gfrm,handangle = sl
                lasol = r.ik(afrm, joints=jts)[0]
                lgsol = r.ik(gfrm, joints=jts)[0]
                langle = handangle
                break
            except:
                continue
        jts = 'rarm'
        for sr in srs:
            try:
                afrm,gfrm,handangle = sr
                rasol = r.ik(afrm, joints=jts)[0]
                rgsol = r.ik(gfrm, joints=jts)[0]
                rangle = handangle
                break
            except:
                continue

        duration2 = 1.0 # time taken to move between approach frame to grasp frame
        plan_and_execute(q0, lasol, rasol, duration)
        execute(lgsol, rgsol, duration2)

        r.grasp2(langle, joints='lhand')
        r.grasp2(rangle, joints='rhand')
        sync(joints='all', duration=graspduration)

        if grabFlag:
            grab(hand='left')
            grab(hand='right')
        else:
            release(hand='left')
            release(hand='right')

        execute(lasol, rasol, duration2)
    move_lr2 = move_lr2_

    def move_(frame, torsoangle, handangle, duration, hand='right'):
        jts0 = 'rarm' if hand == 'right' else 'larm'
        if torsoangle != None:
            jts0 = 'torso_' + jts0

        q0 = r.get_joint_angles(joints=jts0)

        if torsoangle != None:
            r.set_joint_angle(0, torsoangle)
        if frame != None:
            jts = 'rarm' if hand == 'right' else 'larm'
            r.set_joint_angles(r.ik(frame, joints=jts)[0], joints=jts)

        q1 = r.get_joint_angles(joints=jts0)
        traj = pl.make_plan(q0, q1, joints=jts0)
        exec_traj(traj, joints=jts0, duration=duration)

        if handangle != None:
            jts = 'rhand' if hand == 'right' else 'lhand'
            r.grasp2(handangle, joints=jts)
            sync(joints=jts, duration=graspduration)
    move = move_

    def move2_(ss, torsoangle, duration, grabFlag=True, hand='right'):
        jts0 = 'rarm' if hand == 'right' else 'larm'

        def plan_and_execute(q0, sol, duration):
            r.set_joint_angles(sol, joints=jts0)
            jts = 'torso_'+jts0
            q1 = r.get_joint_angles(joints=jts)
            traj = pl.make_plan(q0, q1, joints=jts)
            exec_traj(traj, joints=jts, duration=duration)

        def execute(sol, duration):
            r.set_joint_angles(sol, joints=jts0)
            sync(duration=duration)

        q0 = r.get_joint_angles(joints='torso_'+jts0)

        if torsoangle != None:
            r.set_joint_angle(0, torsoangle)

        for s in ss:
            try:
                afrm,gfrm,handangle = s
                asol = r.ik(afrm, joints=jts0)[0]
                gsol = r.ik(gfrm, joints=jts0)[0]
                angle = handangle
                break
            except:
                continue

        duration2 = 1.0 # time taken to move between  approach frame to grasp frame
        plan_and_execute(q0, asol, duration)
        execute(gsol, duration2)

        handjts = 'rhand' if hand == 'right' else 'lhand'
        r.grasp2(angle, joints=handjts)
        sync(duration=graspduration)

        if grabFlag:
            grab(hand=hand)
        else:
            release(hand=hand)

        execute(asol, duration2)
    move2 = move2_

    def show_frame_(frm, name='frame0'):
        env.delete_object(name)
        bx = visual.box(length=10, height=10, width=10, color=(1,0,1))
        obj = PartsObjectWithName(vbody=bx,name=name)
        obj.vframe.resize(60.0)
        env.insert_object(obj, frm, env.get_world())
    show_frame = show_frame_

    def show_traj_(sts, joints='rarm', name='traj0'):
        env.delete_object(name)
        traj = CoordinateObjects(name)
        for st in sts:
            r.set_joint_angles(st.avec, joints=joints)
            if re.match('.*rarm$', joints) or joints == 'all' or joints == 'torso_arms':
                f = r.fk('right')
                traj.append(f)
            if re.match('.*larm$', joints) or joints == 'all' or joints == 'torso_arms':
                f = r.fk('left')
                traj.append(f)
        env.insert_object(traj, FRAME(), env.get_world())
    show_traj = show_traj_

    def show_tree_():
        show_traj(pl.T_init, name='traj0')
        show_traj(pl.T_goal, name='traj1')
    show_tree = show_tree_

    # Trajectory execution:
    #  Currently just send the last waypoint to the controller,
    #  because smooth execution of a multi-waypoints trajectory.

    def exec_traj_(sts, joints='rarm', name='traj0', duration=4.0):
        if rr == None:
            show_traj(sts, joints=joints, name=name)
        else:
            q = sts[-1].avec
            r.set_joint_angles(q, joints=joints)
            sync(duration=duration)

    # def exec_traj_(traj, duration=0.05, joints='rarm', use_armcontrol=False, draw_trajectory=True):
    #     def robot_relative_traj(traj):
    #         T = -r.get_link('WAIST_Link').where()
    #         qs = [x.avec for x in traj]
    #         ps = []
    #         for q in qs:
    #             r.set_joint_angles(q, joints=joints)
    #             ps.append(T*r.get_link('RARM_JOINT5_Link').where())
    #         return ps

    #     name = 'last_trajectory'
    #     env.delete_object(name)
    #     frames = CoordinateObjects(name)

    #     if rr:
    #         duration = 0.15

    #     if use_armcontrol:
    #         rr.send_trajectory(robot_relative_traj(traj), duration=duration)
    #     else:
    #         for st in traj:
    #             r.set_joint_angles(st.avec, joints=joints)
    #             sync(duration=duration, joints=joints, waitkey=False)

    #             if draw_trajectory:
    #                 if re.match('.*rarm$', joints) or joints == 'torso_arms' or joints == 'all':
    #                     f = r.fk('right')
    #                     frames.append(f)
    #                 if re.match('.*larm$', joints) or joints == 'torso_arms' or joints == 'all':
    #                     f = r.fk('left')
    #                     frames.append(f)
    #         env.insert_object(frames, FRAME(), env.get_world())
    exec_traj = exec_traj_

    def obj_in_hand_(hand='right'):
        prefix = 'R' if hand == 'right' else 'L'
        handlinks = [r.get_link('%sHAND_JOINT%d_Link'%(prefix, n)) for n in [1,3]]
        for obj in env.collidable_objects:
            for l in handlinks:
                if in_collision_pair_parts(l, obj, {}):
                    warn('grab %s'%obj)
                    return obj
        warn('failed to grab')
        return None
    obj_in_hand = obj_in_hand_

    def grab_(hand='right'):
        obj = obj_in_hand(hand=hand)
        if obj == None:
            return False
        r.grabbed_obj[hand] = obj
        affix(obj, hand=hand)
        r.grasp_collision_object(obj, hand=hand)
        for obj2 in env.collidable_objects:
            if obj.name != obj2.name:
                r.add_collision_pair(obj2, obj)
        return True
    grab = grab_

    def release_(hand='right'):
        obj = r.grabbed_obj[hand]
        if obj == None:
            return False
        warn('release %s'%obj)
        r.grabbed_obj[hand] = None
        unfix(obj, hand=hand)
        r.release_collision_object(obj, hand=hand)
        # for obj2 in env.collidable_objects:
        #     if obj.name != obj2.name:
        #         r.remove_collision_pair(obj2, obj)
        return True
    release = release_
