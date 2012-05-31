# -*- coding: utf-8 -*-
# This file defines functions common in all HIRONX instances
# Parameters dependent on each robot instance should be defined in hironx_params.py

from robot import *
import libik_hiro as ikfast
import hironx_params
import copy as traditional_copy

def width2angle(width):
    th = asin(((width/2.0) - 15) / 42)
    # js = [th, -th, -th, th]
    return th

def grasppose(angle):
    dangle = 4*pi/180
    return [(angle-dangle),-angle,-(angle-dangle),angle]


class HiroNx(VRobot):
    def __init__(self,
                 wrldir,
                 scale = 1000.0,
                 name = 'HIRO-NX',
                 forcesensors=True
                 ):

        # predefined poses
        self.poses = {
            'init' : array([0,0,0,
                            -0.010463, 0.0, -1.745329, 0.265277, 0.164054, 0.055845,
                            0.010463, 0.0, -1.745329, -0.265277, 0.164054, -0.055845,
                            0.015708, -0.010472, -0.010472, 0.006981,
                            0.015708, -0.010472, -0.010472, 0.006981]),
            'prepare_right' : array([0.25, -0.4, 1.1,
                                     -0.4, -0.3, -2.1, 0.286, 0.486, 0.695,
                                     0.010, 0.000, -1.745, -0.265, 0.164, -0.056,
                                     0, 0, 0, 0,
                                     0, 0, 0, 0]),
            'prepare' : [0, 0, 1.1,
                         -0.4, -0.3, -2.1, 0.286, 0.486, 0.695,
                         0.4, -0.3, -2.1, -0.286, 0.486, -0.695,
                         0.8, -0.1, -0.8, 0.1,
                         0.8, -0.1, -0.8, 0.1]
            }

        self.hand_poses = {
            'open' : array([0.6, -0.1, -0.6, 0.1]),
            'open2' : array([0.8, -0.1, -0.8, 0.1]),
            'close' : array([0.4, -0.4, -0.4, 0.4]),
            'close2' : array([0.3,-0.3, -0.3, 0.3]),
            'push' : array([-pi/6,2*pi/3+0.1,pi/6,-2*pi/3+0.1])
            }

        # load fixed transforms dependent on each robot instance
        for k,v in hironx_params.params.items():
            setattr(self, k, v)

        if not forcesensors:
            self.Tikoffset.vec[0] -= 59

        # deprecated, for compatibility
        self.Twrist_ef = self.Trwrist_ef

        if forcesensors:
            mainfile = 'main.wrl'
        else:
            mainfile = 'main_no_force_sensors.wrl'
        VRobot.__init__(self, wrldir, mainfile, scale, name)

        # safe(soft) joint limits
        for i,lims in enumerate([deg2rad(x) for x in [(-90,90),(-70,70),(-20,70),
                                                      (-80,80),(-86,29),(-143,-12),(-86,86),(-95,95),(-110,110),
                                                      (-80,80),(-86,29),(-143,-12),(-86,86),(-95,95),(-110,110),
                                                      (-109,68),(-150,90),(-109,68),(-150,90),
                                                      (-109,68),(-150,90),(-109,68),(-150,90)]]):
            j = self.joints[i]
            j.sllimit,j.sulimit = lims

        # maximum workspace movement in [mm]
        for j,w in zip(self.joints, [800,200,200,
                                     700,650,320,120,140,50,
                                     700,650,320,120,140,50,
                                     20,20,20,20,
                                     20,20,20,20]):
            j.weight = w

        # attach sensors
        rhandcam = SensorObject(name='rhandcam')
        rhandcam.affix(self.get_joint('RARM_JOINT5'), self.Trh_cam)
        self.sensors.append(rhandcam)
        lhandcam = SensorObject(name='lhandcam')
        lhandcam.affix(self.get_joint('LARM_JOINT5'), self.Tlh_cam)
        self.sensors.append(lhandcam)
        kinectdepth = SensorObject(name='kinectdepth')
        kinectdepth.affix(self.get_joint('HEAD_JOINT1'), self.Thd_kinectdepth)
        self.sensors.append(kinectdepth)

    def prepare_right(self):
        self.set_joint_angles(self.poses['prepare_right'])

    def get_joint_angles(self, joints='all'):
        if joints == 'rarm':
            js = self.joints[3:9]
        elif joints == 'larm':
            js = self.joints[9:15]
        elif joints == 'torso_rarm':
            js = [self.joints[0]]+self.joints[3:9]
        elif joints == 'torso_larm':
            js = [self.joints[0]]+self.joints[9:15]
        elif joints == 'torso_arms':
            js = [self.joints[0]]+self.joints[3:15]
        elif joints == 'rhand':
            js = self.joints[15:19]
        elif joints == 'lhand':
            js = self.joints[19:23]
        elif joints == 'head':
            js = self.joints[1:3]
        else:
            js = self.joints
        return [j.angle for j in js]

    def set_joint_angles(self, ths, joints='all', flush=True, check_collision=False):
        if joints == 'rarm':
            js = self.joints[3:9]
        elif joints == 'larm':
            js = self.joints[9:15]
        elif joints == 'torso_rarm':
            js = [self.joints[0]]+self.joints[3:9]
        elif joints == 'torso_larm':
            js = [self.joints[0]]+self.joints[9:15]
        elif joints == 'torso_arms':
            js = [self.joints[0]]+self.joints[3:15]
        elif joints == 'rhand':
            js = self.joints[15:19]
        elif joints == 'lhand':
            js = self.joints[19:23]
        elif joints == 'head':
            js = self.joints[1:3]
        else:
            js = self.joints

        if check_collision:
            ths_old = self.get_joint_angles(joints=joints)

        for j,th in zip(js,ths):
            j.angle = th

        if flush:
            self.refresh()

        if check_collision and self.in_collision():
            warn('collision detected')
            for j,th in zip(js, ths_old):
                j.angle = th
            if flush:
                self.refresh()

    #
    #  deprecated
    #
    # def set_arm_joint_angles(self, ths, arm='right', flush=True):
    #     '''@rtc_interface: void SetArmJointAngles(in DoubleSequence, in String, in bool)
    #     '''
        
    #     rarm = self.check_right_or_left(arm)
        
    #     if len(ths) != 6:
    #         print "the length of joint angles is wrong"
    #         return
        
    #     if rarm:
    #         jointid_offset = 3
    #     else:
    #         jointid_offset = 9

    #     for i in range(len(ths)):            
    #         self.set_joint_angle(i+jointid_offset, ths[i], flush=False)
    #     if flush:
    #         self.refresh()

    # def get_arm_joint_angles(self, arm='right'):
    #     rarm = self.check_right_or_left(arm)
        
    #     if rarm:
    #         return self.get_joint_angles()[3:9]
    #     else:
    #         return self.get_joint_angles()[9:15]            

    # def get_hand_joint_angles(self, hand='right'):
    #     rhand = self.check_right_or_left(hand)

    #     if rhand:
    #         return self.get_joint_angles()[15:19]
    #     else:
    #         return self.get_joint_angles()[19:23]

    # def set_hand_joint_angles(self, angles, hand='right'):
    #     rhand = self.check_right_or_left(hand)
    #     if len(angles) != 4:
    #         print "the length of joint angles is wrong"
    #         return
        
    #     js = self.get_joint_angles()
    #     if rhand:
    #         js[15:19] = angles
    #     else:
    #         js[19:23] = angles
    #     self.set_joint_angles(js)

    def init_clink_pairs(self):
        blacklist = [(0,2),(0,3),(0,4),(0,5),(0,9),(0,10),(0,11),
                     (2,3),(2,9),
                     (6,8),(6,15),(6,17),
                     (7,15),(7,17),
                     (8,15),(8,17),
                     (12,14),(12,19),(12,21),
                     (13,19),(13,21),
                     (14,19),(14,21),
                     (15,17),(15,18),(16,18),
                     (19,21),(19,22),(20,22)
                     ]

        self.clink_pairs = []
        n = len(self.joints)
        for i in range(n):
            for j in range(i+2, n):
                if not (i, j) in blacklist:
                    self.clink_pairs.append((self.joints[i].link,
                                             self.joints[j].link))

    def add_collision_object(self, obj):
        lnknames = ['ARM_JOINT2_Link', 'ARM_JOINT3_Link',
                    'ARM_JOINT4_Link', 'ARM_JOINT5_Link',
                    'HAND_JOINT0_Link', 'HAND_JOINT1_Link',
                    'HAND_JOINT2_Link', 'HAND_JOINT3_Link']
        for lnknm in ['R'+s for s in lnknames]:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)
        for lnknm in ['L'+s for s in lnknames]:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)

    def grasp_collision_object(self, obj, hand='right'):
        for lnknm in ['WAIST_Link', 'CHEST_JOINT0_Link',
                      'HEAD_JOINT0_Link', 'HEAD_JOINT1_Link']:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)
        prefix1 = 'L' if hand == 'right' else 'R'
        prefix2 = 'R' if hand == 'right' else 'L'
        for lnknm in [prefix1+s for s in ['ARM_JOINT0_Link',
                                           'ARM_JOINT1_Link',
                                           'HAND_JOINT0_Link',
                                           'HAND_JOINT1_Link',
                                           'HAND_JOINT2_Link',
                                           'HAND_JOINT3_Link']]:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)
        for lnknm in [prefix2+s for s in ['HAND_JOINT0_Link',
                                            'HAND_JOINT1_Link',
                                            'HAND_JOINT2_Link',
                                            'HAND_JOINT3_Link']]:
            lnk = self.get_link(lnknm)
            self.remove_collision_pair(lnk, obj)

    def release_collision_object(self, obj, hand='right'):
        for lnknm in ['WAIST_Link', 'CHEST_JOINT0_Link',
                      'HEAD_JOINT0_Link', 'HEAD_JOINT1_Link']:
            lnk = self.get_link(lnknm)
            self.remove_collision_pair(lnk, obj)
        prefix1 = 'L' if hand == 'right' else 'R'
        prefix2 = 'R' if hand == 'right' else 'L'
        for lnknm in [prefix1+s for s in ['ARM_JOINT0_Link',
                                           'ARM_JOINT1_Link',
                                           'HAND_JOINT0_Link',
                                           'HAND_JOINT1_Link',
                                           'HAND_JOINT2_Link',
                                           'HAND_JOINT3_Link']]:
            lnk = self.get_link(lnknm)
            self.remove_collision_pair(lnk, obj)
        for lnknm in [prefix2+s for s in ['HAND_JOINT0_Link',
                                          'HAND_JOINT1_Link',
                                          'HAND_JOINT2_Link',
                                          'HAND_JOINT3_Link']]:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)


    def fk_fast_rarm(self, ths, scl=1e+3):
        rot,trans = ikfast.fk(ths)
        m = MATRIX(mat=rot)
        v = VECTOR(vec=[scl*x for x in trans])
        return FRAME(mat=m, vec=v)

    def fk(self, arm='right', scl=1e+3):
        rarm = self.check_right_or_left(arm)

        if rarm:
            ths = self.get_joint_angles()[3:9]
            f = self.fk_fast_rarm(ths)
            f = f*(-self.Tikoffset) ###
            Twc = self.links[2].where()
            return Twc*f
        else:
            ths = self.get_joint_angles()[9:15]
            for i in [0,3,5]:
                ths[i] = -ths[i]
            f = self.fk_fast_rarm(ths)
            f = f*(-self.Tikoffset) ###
            x,y,z = f.vec
            a,b,c = f.mat.abc()
            f = FRAME(xyzabc=[x,-y,z,-a,b,-c])
            Twc = self.links[2].where()
            return Twc*f

    def ik_fast_rarm(self, efframe, scl=1e-3):
        efframe = efframe*self.Tikoffset ###
        return ikfast.ik(efframe.mat, [scl * x for x in efframe.vec])

    def ik(self, frms, joints='rarm', scl=1e-3, flush=True):
        def reverse_frame(frm):
            x,y,z = frm.vec
            a,b,c = frm.mat.abc()
            return FRAME(xyzabc=[x,-y,z,-a,b,-c])

        arm,use_waist = parse_joints_flag(joints)

        rarm = self.check_right_or_left(arm)

        if frms.__class__ == FRAME:
            frms = [frms]

        # evaluate only arm pose
        # q_reference = self.poses['prepare'][3:9]
        q_reference = self.get_joint_angles()[3:9]

        if use_waist:
            th_orig = self.get_joint_angle(0) # save the waist yaw angle
            waist_yaw_samples = linspace(-1.0, 1.0, 11)
            sols = []
            for th in waist_yaw_samples:
                # 20120526
                Twc = self.links[2].where()
                Twc.mat = MATRIX(Twc.mat, angle=th, axis=Twc.mat.row(2))
                # self.set_joint_angle(0, th, flush=flush)
                # Twc = self.links[2].where()
                frms2 = [(-Twc)*frm for frm in frms]
                if not rarm:
                    frms2 = map(reverse_frame, frms2)

                sols += [(th,x) for x in reduce(operator.add,
                                                map(lambda frm: self.ik_fast_rarm(frm),
                                                    frms2))]
            sols2 = self.eval_solutions(sols, q_reference, use_waist=use_waist)
            if not rarm:
                for sol in sols2:
                    for i in [0,3,5]:
                        sol[1][i] = -sol[1][i]

            self.set_joint_angle(0, th_orig, flush=flush) # restore the waist yaw angle

            sols3 = []
            for th,avec in sols2:
                avec.insert(0, th)
                sols3.append(avec)
            return sols3

        else:
            Twc = self.links[2].where()
            frms = map(lambda frm: (-Twc)*frm, frms)
            if not rarm:
                frms = map(reverse_frame, frms)

            sols = reduce(operator.add,
                          map(lambda frm: self.ik_fast_rarm(frm),
                              frms))
            sols2 = self.eval_solutions(sols, q_reference, use_waist=use_waist)
            if not rarm:
                for sol in sols2:
                    for i in [0,3,5]:
                        sol[i] = -sol[i]
            return sols2

    def weighted_qdist(self, q1, q2):
        return weighted_L1dist(q1, q2, [1.0, 0.8, 0.3, 0.1, 0.1, 0.02])
    
    def within_joint_limit(self, q):
        for i in range(6):
            jnt = self.joints[i+3]
            if q[i] < jnt.sllimit or q[i] > jnt.sulimit:
                # print 'Limits: %d,%f,%f,%f'%(i+3,q[i],jnt.sllimit,jnt.sulimit)
                return False
        return True

    def eval_solutions(self, sols, q_reference, use_waist=False):
        if use_waist:
            sols2 = [x for x in sols if self.within_joint_limit(x[1])]
            sols2.sort(lambda q1,q2: cmp(self.weighted_qdist(q_reference, q1[1]),
                                         self.weighted_qdist(q_reference, q2[1])))
        else:
            sols2 = [x for x in sols if self.within_joint_limit(x)]
            sols2.sort(lambda q1,q2: cmp(self.weighted_qdist(q_reference, q1),
                                         self.weighted_qdist(q_reference, q2)))
        return sols2

    def check_right_or_left(self, arm):
        if arm == 'right':
            return True
        else:
            return False

    def grasp(self, width, hand='right'):
        jts = hand[0] + 'hand'
        th = asin(((width/2.0) - 15) / 42)
        js = [th, -th, -th, th]
        self.set_joint_angles(js, joints=jts)

    def grasp2(self, angle, joints='rhand'):
        self.set_joint_angles(grasppose(angle), joints=joints)

    # target frame
    def lookat(self, tf, flush=True, torso_angle=None):
        if flush==False and torso_angle == None:
            print "lookat(): set torso_angle if you do not use flushing."
            return ;
        else:
            self.refresh()
        #
        hf = self.get_link("HEAD_JOINT1_Link").where()
        v = tf.vec - hf.vec
        angles = self.get_joint_angles()
        if flush==False and angles == None:        
            torso_angle = angles[0]
        angles[1] =  math.atan2(v[1],v[0]) - torso_angle # subtract torso angle
        angles[2] = -math.atan2(v[2],v[0])
        #
        if flush == True:
            self.set_joint_angles(angles)
        #
        return angles[1:3]

    def lookat_hand(self,hand="right", flush=True, tang=None):
        lname = "RARM_JOINT5_Link" if hand[0]=="r" else "LARM_JOINT5_Link"
        return self.lookat(self.get_link(lname).where(), flush=flush, torso_angle=tang)
 
