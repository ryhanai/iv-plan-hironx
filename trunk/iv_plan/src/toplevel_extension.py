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
    global show_frame, show_traj, show_tree, exec_traj
    global obj_in_hand, grab, release

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

    def exec_traj_(sts, joints='rarm', name='traj0'):
        show_traj(sts, joints=joints, name=name)
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
