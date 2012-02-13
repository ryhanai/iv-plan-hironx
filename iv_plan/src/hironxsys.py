# -*- coding: utf-8 -*-

import socket
import pickle
import operator

from numpy import *
from ivutils import *
import rtc_helper

class HiroNxSystem:
    def __init__(self, portdefs):
        self.joint_states = zeros(23)
        self.portdefs = portdefs
        self.connect()

    def connect(self):
        portdef = self.portdefs['controller']
        self.motionsvc = rtc_helper.get_service(portdef)

        portdef = self.portdefs['jointstat']
        self.jstt_port = rtc_helper.get_port(portdef)

        #h_fc = get_handle('force.rtc', ns)
        #self.force_port = h_fc.outports['dataout']

        # h_rh = get_handle('HIRONXController(Robot)0.rtc', ns)
        # self.jstt_port = h_rh.outports['q']
        # rospy.Subscriber('/hiro/joint_state', JointState, self.update_joint_state)

    def read_joint_state(self):
        data = self.jstt_port.read()
        secs = data.tm.sec
        nsecs = data.tm.nsec
        self.joint_states = reduce(operator.__concat__, data.qState)
        # self.joint_states = data.data
        # velocity = reduce(operator.__concat__, data.dqState)

    def get_force_data(self):
        return self.force_port.read()

    def get_joint_angles(self):
        self.read_joint_state()
        return self.joint_states

    def send_goal(self, joint_angles, duration, wait=True):
        if joint_angles[0].__class__ == list:
            q = self.get_joint_angles()
            for i,q_ in enumerate(joint_angles):
                if q_ != []:
                    if i == 0:
                        q[0:3] = q_
                    if i == 1:
                        q[3:9] = q_
                    if i == 2:
                        q[9:15] = q_
                    if i == 3:
                        q[15:19] = q_
                    if i == 4:
                        q[19:23] = q_
            joint_angles = q
        joint_angles = [rad2deg(x) for x in joint_angles]+[0]
        print joint_angles
        self.motionsvc.ref.movePTPJointAbsSeq([joint_angles], [duration])

    def send_trajectory(self, ps, duration=2.0, arm='right'):
        traj = []
        s = 0.001
        for p in ps:
            x,y,z = p.vec
            a,b,c = euler_from_matrix(p.mat, axes='sxyz')
            # p.mat.abc() = euler_from_matrix(p.mat, axes='szyx')
            traj.append(([s*x,s*y,s*z,a,b,c], duration))
        msg = ('trajectory', traj)
        self.send_msg(msg, timeout=30.0)

    def send_msg(self, msg, timeout = 10.0):
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if timeout > 0.0:
            soc.settimeout(timeout)
        soc.connect((self.seq_proxy_address, self.seq_proxy_port))
        soc.sendall(pickle.dumps(msg))
        msgstr = soc.recv(4096)
        msg = pickle.loads(msgstr)
        print msg
        soc.close()


    # def send_goal(self, joint_angless, duration, wait=True):
    #     # convert from numpy.float64 to float
    #     goal = map(lambda joint_angles: map(lambda x: float(x), joint_angles), joint_angless)
    #     msg = ('goal', goal, duration, wait)
    #     self.send_msg(msg)

    # def __init__(self, nameserver, seq_proxy_host='150.29.146.166', seq_proxy_port=10103):
    # def __init__(self, nameserver, seq_proxy_host='192.168.128.253', seq_proxy_port=10103):
    #     try:
    #         self.seq_proxy_address = socket.gethostbyname(seq_proxy_host)
    #     except:
    #         self.seq_proxy_address = seq_proxy_host
    #     self.seq_proxy_port = seq_proxy_port
    #     self.joint_states = zeros(23)
