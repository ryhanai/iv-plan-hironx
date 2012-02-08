# -*- coding: utf-8 -*-

from hiro_controller import *

if ros_available:
    import rospy
    import tf
    from sensor_msgs.msg import JointState
    from ar_pose.msg import ARMarkers
    import geometry_msgs.msg
    from tf.transformations import *
##

class RealHIRO(HIROController):
    def __init__(self, nameserver):
        HIROController.__init__(self, nameserver)
        self.rhand_pose_markers = []
        self.lhand_pose_markers = []
        self.kinect_centers = []
        self.kinect_pose_markers = []

    def connect(self):
        HIROController.connect(self)
        if ros_available:
            rospy.init_node('motion_planner')
            rospy.Subscriber('/hiro/rhand/ar_pose_marker', ARMarkers, self.update_rhand_cam)
            rospy.Subscriber('/hiro/lhand/ar_pose_marker', ARMarkers, self.update_lhand_cam)
            rospy.Subscriber('/calc_center', geometry_msgs.msg.PoseStamped, self.update_calc_center)
            rospy.Subscriber('/ar_pose_marker', ARMarkers, self.update_kinect_AR)
            self.listener = tf.TransformListener()

        # self.h_leyecap = get_handle('leye_capture.rtc', ns)
        # activate([self.h_leyecap])
        # self.h_reyecap = get_handle('reye_capture.rtc', ns)
        # self.h_rhandcap = get_handle('rhand_capture.rtc', ns)
        # activate([self.h_leyecap, self.h_reyecap])

    def __del__(self):
        #deactivate([self.h_leyecap, self.h_reyecap])
        deactivate([self.h_leyecap])

    def update_rhand_cam(self, msg):
        if len(msg.markers) > 0:
            self.rhand_pose_markers = msg.markers
        #     marker = msg.markers[0]
        #     print 'stamp ', 
        #     print marker.header.stamp
        #     print 'frame_id ',
        #     print marker.header.frame_id
        #     print 'id ',
        #     print marker.id
        #     print 'position ',
        #     print marker.pose.pose.position
        #     print 'orientation ',
        #     print marker.pose.pose.orientation

    def update_lhand_cam(self, msg):
        if len(msg.markers) > 0:
            self.lhand_pose_markers = msg.markers

    def update_kinect_AR(self, msg):
        if len(msg.markers) > 0:
            self.kinect_pose_markers = msg.markers

    def update_calc_center(self, msg):
        def near(p1,p2,eps=0.02):
            return linalg.norm([p1.x-p2.x,p1.y-p2.y,p1.z-p2.z]) < eps
        # point = msg.pose.position
        # orientation = msg.pose.orientation
        # header = msg.header

        # append if the new observation is different
        # from any of registered ones.
        for cs in self.kinect_centers:
            if near(msg.pose.position, cs.pose.position):
                return

        self.kinect_centers.append(msg)

        # # discard old observations
        # tnow = rospy.Time.now().to_sec()
        # for cs in self.kinect_centers:
        #     if tnow - .stamp.to_sec() > thre:

    def get_tf(self, from_frm='/leye', to_frm='/checkerboard'):
        tfs = {}
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(from_frm, to_frm,
                                           now, rospy.Duration(3.0))
            return self.listener.lookupTransform(from_frm, to_frm, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.Exception):
            return None

    def detect(self, wait=True, camera='leye', thre=1.5):
        '''camera := "leye" | "rhand" | "lhand", returns the recent recognition result within thre[sec] for hand cameras'''
        if camera == 'leye':
            rate = rospy.Rate(2.0)
            if wait:
                while not rospy.is_shutdown():
                    tf = self.get_tf()
                    # if tfs.has_key(camera):
                    #     tf = tfs[camera]
                    if not tf == None:
                        break
                    rate.sleep()
            else:
                tf = self.get_tf('/leye', '/checkerboard')
                if tf == None:
                    return None

            (trans, rot) = tf
            Tcam_obj = FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                             vec=VECTOR(vec=(1000.0*array(trans)).tolist()))
            return Tcam_obj

        elif camera == 'kinect_rgb':
            rate = rospy.Rate(2.0)
            if wait:
                while not rospy.is_shutdown():
                    tf = self.get_tf('/openni_rgb_optical_frame', '/checkerboard_k')
                    if not tf == None:
                        break
                    rate.sleep()
            else:
                tf = self.get_tf('/openni_rgb_optical_frame', '/checkerboard_k')
                if tf == None:
                    return None

            (trans, rot) = tf
            Tcam_obj = FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                             vec=VECTOR(vec=(1000.0*array(trans)).tolist()))
            return Tcam_obj

        elif camera == 'kinect_point_center':
            tnow = rospy.Time.now().to_sec()
            for cs in self.kinect_centers:
                if tnow - cs.header.stamp.to_sec() > thre:
                    self.kinect_centers.remove(cs)

            return [FRAME(mat=MATRIX(mat=quaternion_matrix([0,0,0,0])[0:3,0:3].tolist()),
                          vec=VECTOR(1000.0*cs.pose.position.x,1000.0*cs.pose.position.y,1000.0*cs.pose.position.z)) for cs in self.kinect_centers]

            # p = self.kinect_center_point
            # trans = 1000.0 * array([p.x, p.y, p.z])
            # rot = [0,0,0,0]


        elif camera == 'rhand' or camera == 'lhand':
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

            if camera == 'rhand':
                return filter(None, [parse_marker(m) for m in self.rhand_pose_markers])
            else:
                return filter(None, [parse_marker(m) for m in self.lhand_pose_markers])

        elif camera == 'kinect_AR':
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

            return filter(None, [parse_marker(m) for m in self.kinect_pose_markers])

        else:
            print 'specified camera is not supported'
