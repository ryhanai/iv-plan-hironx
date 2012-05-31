# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
import set_env
from ivutils import *
from hiro_controller import *

import rospy
from ar_pose.msg import ARMarkers
from tf.transformations import *

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

                return (marker.id,
                        FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                              vec=VECTOR(vec=(trans.tolist()))))

        if camera == 'lhand':
            return filter(None, [parse_marker(m) for m in self.lhand_markers])
        else:
            return filter(None, [parse_marker(m) for m in self.rhand_markers])


rr = MyRobotInterface(set_env.nameserver)
rr.connect()
