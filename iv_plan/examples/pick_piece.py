# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
import set_env
from ivutils import *
from viewer import *
import scene_objects
from robot import *
from mplan_env import *


from hiro_controller import *

import rospy
from ar_pose.msg import ARMarkers
from tf.transformations import *

def sync(duration=4.0, joints='all', wait=True, waitkey=True):
    '''synchronize the real robot with the model in "duration" [sec]'''
    if rr:
        js = r.get_joint_angles()
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
            time.sleep(duration)


class MyRobotInterface(HIROController):
    def __init__(self, nameserver):
        HIROController.__init__(self, nameserver)
        self.lhand_markers = []

    def connect(self):
        HIROController.connect(self)
        rospy.init_node('pick_piece')
        rospy.Subscriber('/hiro/lhand/ar_pose_marker', ARMarkers, self.lhand_callback)

    def lhand_callback(self, msg):
        if len(msg.markers) > 0:
            self.lhand_markers = msg.markers

    def recognize(self, camera='lhand', thre=1.5):
        def parse_marker(marker):
            if rospy.Time.now().to_sec() - marker.header.stamp.to_sec() > thre:
                return None
            else:
                p = marker.pose.pose.position
                trans = 1000.0 * array([p.x, p.y, p.z])
                q = marker.pose.pose.orientation
                rot = [q.x, q.y, q.z, q.w]

                return (#marker.header.frame_id,
                        marker.id,
                        FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                              vec=VECTOR(vec=(trans.tolist()))))

        if camera == 'lhand':
            return filter(None, [parse_marker(m) for m in self.lhand_markers])
        else:
            return filter(None, [parse_marker(m) for m in self.rhand_markers])


def show_frame(frm, name='frame0'):
    env.delete_object(name)
    bx = visual.box(length=10, height=10, width=10, color=(1,0,1))
    obj = PartsObjectWithName(vbody=bx,name=name)
    obj.vframe.resize(60.0)
    env.insert_object(obj, frm, env.get_world())

def grasp_frame(f):
    return f * FRAME(xyzabc=[0,-30,-2,0,0,-pi/2])

def world_frame(Tcam_piece, js):
    r.set_joint_angles(js)
    return r.get_link('LARM_JOINT5_Link').where() * r.Tlh_cam * Tcam_piece

def move_arm(f, jts, duration=2.0):
    r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
    sync(joints=jts, duration=duration)

def frame_by_id(res, n):
    return [x for x in res if x[0] == n][0][1]

def pick(f, hand='left'):
    gf = f * (-r.Twrist_ef)
    af = f * FRAME(xyzabc=[0,0,30,0,0,0]) * (-r.Twrist_ef)

    jts = hand[0]+'arm'
    move_arm(af, jts)
    move_arm(gf, jts)
    r.grasp(width=24, hand=hand)
    sync(joints=hand[0]+'hand', duration=0.3)
    move_arm(af, jts)

def place(f, hand='left'):
    gf = f * FRAME(xyzabc=[0,0,30,0,0,0]) * (-r.Twrist_ef)
    af = f * FRAME(xyzabc=[0,0,60,0,0,0]) * (-r.Twrist_ef)

    jts = hand[0]+'arm'
    move_arm(af, jts)
    move_arm(gf, jts)
    r.grasp(width=35, hand=hand)
    sync(joints=hand[0]+'hand', duration=0.3)
    r.grasp(width=60, hand=hand)
    sync(joints=hand[0]+'hand', duration=0.3)
    move_arm(af, jts)

def detect_pose():
    tblheight = 700
    fsoffset = 59
    detectpos_dual = [(180,-35),(180,155)]

    r.prepare(width=80)
    jts = 'rarm'
    x,y = detectpos_dual[0]
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290,0,-pi/2,0])
    r.set_joint_angles(r.ik(f, jts)[0], joints=jts)
    jts = 'larm'
    x,y = detectpos_dual[1]
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290,0,-pi/2,0])
    r.set_joint_angles(r.ik(f, jts)[0], joints=jts)
    sync()

def demo():
    res = rr.recognize()
    if len(res) < 2:
        warn('recognition failed')
        return
    print res
    q = rr.get_joint_angles()
    frame2 = world_frame(grasp_frame(frame_by_id(res, 2)), q)
    frame4 = world_frame(grasp_frame(frame_by_id(res, 7)), q)
    show_frame(frame2)
    show_frame(frame4)
    pick(frame2)
    place(frame4)


env = MPlanEnv()
env.load_scene(scene_objects.table_scene())

r = VHIRONX(ivpkgdir+'/iv_plan/externals/models/HIRONX_110822/')
env.insert_robot(r)
r.go_pos(-150, 0, 0)

rr = MyRobotInterface(set_env.nameserver)
rr.connect()

