#!/usr/bin/env python
import rospy
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial
from cs4752_proj2.msg import *
from cs4752_proj2.srv import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
import tf
import tf2_ros
from tf.transformations import *
from copy import deepcopy

def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

class controller() :
    def __init__(self):
        rospy.init_node('controller')
        loginfo("Initialized node Controller")
        
        # baxter_interface.RobotEnable(CHECK_VERSION).enable()

        # self.left = baxter_interface.Limb('left')
        # self.left_kin = baxter_kinematics('left')

        # rospy.wait_for_service("/move_end_effector_trajectory")
        # self.joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)
        # self.tool_trajectory = rospy.ServiceProxy("/move_tool_trajectory", JointAction)
        # loginfo("Initialized Joint Action Server Proxy")
        # rospy.wait_for_service("/end_effector_position")
        # self.position_server = rospy.ServiceProxy("/end_effector_position", EndEffectorPosition)
        # loginfo("Initialized position server proxy")
        # rospy.wait_for_service("/tool_position")
        # self.tool_position_server = rospy.ServiceProxy("/tool_position", EndEffectorPosition)
        # loginfo("Initialized tool_position server proxy")

        # self.point_publisher = rospy.Publisher('/pt_stamped', PointStamped, queue_size=10)
        # self.marker_array_pub = rospy.Publisher('/test_marker_array', MarkerArray, queue_size=10)
        self.pose_publisher = rospy.Publisher('/test_pose_array', PoseArray, queue_size=10)
        self.calib_pts = []

        # self.tf_br = tf.TransformBroadcaster()
        self.tf_br = tf2_ros.TransformBroadcaster()

        self.calibrate_plane()

        rospy.Subscriber("/plane_traj", Trajectory, self.plane_trajCb, queue_size=10000)


        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            self.sendTransform()
            rate.sleep()

        # rospy.spin()

    def sendTransform(self):
        try:
            self.plane_translation
            self.plane_rotation
        except NameError:
            self.plane_translation = np.identity(4)
            self.plane_rotation = np.identity(4)
            # self.plane_rotation = self.matrix_from_euler(0,0,-45)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base"
        t.child_frame_id = "plane_frame"
        T = translation_from_matrix(self.plane_translation)
        t.transform.translation.x = T[0]
        t.transform.translation.y = T[1]
        t.transform.translation.z = T[2]
        q = quaternion_from_matrix(self.plane_rotation)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_br.sendTransform(t)
        
        # self.tf_br.sendTransform(
        #     translation_from_matrix(self.plane_translation), 
        #     quaternion_from_matrix(self.plane_rotation),
        #     rospy.Time.now(), 
        #     "plane_frame", 
        #     "base")

    def create_point_stamped(self, pt, frame_id = 'base'):
        m = PointStamped()
        m.header.frame_id = frame_id
        # m.header.stamp = rospy.Time()
        m.header.stamp = rospy.get_rostime()
        m.point = Point(pt[0],pt[1],pt[2])
        self.point_publisher.publish(m)
        # return m

    def create_marker(self, pt, frame_id = "/base"):
        # m = PointStamped()
        # m.header.frame_id = frame_id
        # # m.header.stamp = rospy.Time()
        # m.header.stamp = rospy.get_rostime()
        # m.point = Point(pt[0],pt[1],pt[2])
        # self.point_publisher.publish(m)
        # # return m

        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()

        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.orientation.w = 1.0

        marker.pose.position = Point(pt[0],pt[1],pt[2])

        return marker

        # print marker.pose.position
        # self.marker_pub.publish(marker)

    def create_marker_array(self, pts, frame_id = "/base"):
        markers = MarkerArray()
        for pt in pts:
            marker = self.create_marker(pt, frame_id=frame_id)
            markers.markers.append(marker)
        self.marker_array_pub.publish(markers)

    def create_pose_array(self, pts, frame_id = "/base"):
        poses = PoseArray()
        poses.header.frame_id = frame_id
        poses.header.stamp = rospy.Time.now()

        for pt in pts:
            pose = Pose()
            pose.position = Point(pt[0],pt[1],pt[2])
            pose.orientation.w = 1.0
            poses.poses.append(pose)

        # global pose_publisher
        # pose_publisher.publish(poses)
        self.pose_publisher.publish(poses)


    def calibrate_plane(self):
        # point_count = 0
        point_pos = []
        # while point_count < 3 :
        #     prompt = "Press Enter when Arm is on the %d plane point" % point_count
        #     cmd = raw_input(prompt)
        #     point_pos.append(self.get_tool_pos())
        #     print point_pos[point_count]
        #     point_count += 1

        # test begin $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        rospy.sleep(2)
        # self.create_point_stamped([1.0,1.0,0.9])
        # self.create_point_stamped([1.0,1.0,0.8])
        # self.create_point_stamped([1.0,1.0,0.7])
        # self.create_point_stamped([1.0,1.0,0.6])

        # pt1 = [0.8,1.0,0.6]
        # pt2 = [0.8,0.7,0.6]
        # pt3 = [0.8,1.0,0.9]

        pt1 = [1.0,1.0,0.6]
        pt2 = [0.8,0.7,0.6]
        pt3 = [.8,1.0,0.3]


        self.calib_pts.append(pt1)
        self.calib_pts.append(pt2)
        self.calib_pts.append(pt3)
        # self.create_marker_array(self.calib_pts)
        self.create_pose_array(self.calib_pts)

        # self.create_pose([1.0,1.0,0.9])
        # self.create_pose([1.0,1.0,0.8])
        # self.create_pose([1.0,1.0,0.7])
        point_pos.append(np.array(pt1))
        point_pos.append(np.array(pt2))
        point_pos.append(np.array(pt3))
        # test end $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

        vec1 = point_pos[1] - point_pos[0]
        vec2 = point_pos[2] - point_pos[0]
        
        self.plane_norm = np.cross(vec1, vec2)
        # plane_origin = np.average(point_pos, axis=0)
        plane_origin = point_pos[0]

        # go left to right from pt1 to pt2 and then up to pt3

        # make sure normal is pointing towards the base
        # if(np.dot(-plane_origin,self.plane_norm) < 0):
        #     self.plane_norm *= -1.

        self.plane_translation = translation_matrix(plane_origin)
        
        x_plane = vec1/np.linalg.norm(vec1)
        y_plane = np.cross(self.plane_norm, vec1)
        y_plane = y_plane/np.linalg.norm(y_plane)
        z_plane = self.plane_norm/np.linalg.norm(self.plane_norm)
        #need rotation to make norm the z vector
        self.plane_rotation = np.array([x_plane, y_plane, z_plane]).T

        # print "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"
        # print np.linalg.det(self.plane_rotation)
        # x = np.array([2,0,0])
        # print np.dot(self.plane_rotation,x)
        # print "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"

        self.plane_rotation = np.append(self.plane_rotation,[[0,0,0]],axis=0)
        self.plane_rotation = np.append(self.plane_rotation,[[0],[0],[0],[1]],axis=1)
        
        print "#################################"
        print "Finished Calibrating Plane"
        print "self.plane_translation"
        print translation_from_matrix(self.plane_translation)
        print "self.plane_rotation"
        print self.plane_rotation
        # print euler_from_matrix(self.plane_rotation)
        print "#################################"

        self.sendTransform()

        

    def get_tool_pos(self):
        tool_vec = self.tool_position_server().position
        tool = np.array([tool_vec.x,tool_vec.y,tool_vec.z])
        # print "******************************************"
        # print "tool"
        # print tool    while point_count < 3 :

        # print "******************************************"
        return tool



    def matrix_from_euler(self, r,p,y,radians=False):
        if not radians:
            a = math.pi/180.
            r *= a
            p *= a
            y *= a

        q = quaternion_from_euler(r, p, y)
        return quaternion_matrix(q)

    def PlaneToBasePoint(self, plane_x,plane_y):
        try:
            self.plane_translation
            self.plane_rotation
        except NameError:
            self.plane_translation = np.identity(4)
            # self.plane_rotation = np.identity(4)
            self.plane_rotation = self.matrix_from_euler(0,0,-45)

        M = np.dot(self.plane_translation, self.plane_rotation)

        plane_coords = np.array([plane_x,plane_y,0,1])
        base_coords = np.dot(M, plane_coords.T)
        base_coords = base_coords[:3]/base_coords[3]
        base_coords.reshape((1, 3))
        # print "base_coords: {0}".format(base_coords)
        return base_coords

    def PlaneToBaseDir(self, plane_x,plane_y):
        try:
            self.plane_rotation
        except NameError:
            # self.plane_rotation = np.identity(4)
            self.plane_rotation = self.matrix_from_euler(0,0,-45)

        plane_dir = np.array([plane_x,plane_y,0,1])
        base_dir = np.dot(self.plane_rotation, plane_dir.T)
        base_dir = base_dir[:3]/base_dir[3]
        base_dir.reshape((1, 3))
        # print "base_dir: {0}".format(base_dir)
        return base_dir

    def plane_trajCb(self, plane_traj_msg):
        T = np.array(plane_traj_msg.times)
        P = np.zeros([0,3])
        V = np.zeros([0,3])

        loginfo("Making position call")
        initial_position = self.position_server().position
        loginfo(initial_position)

        positions = []
        velocities = []
        for i in range(0,len(plane_traj_msg.positions)):
            pp = plane_traj_msg.positions[i]
            wp = self.PlaneToBasePoint(pp.x,pp.y)
            P = np.append(P, [wp], axis=0)
            positions.append(Vector3(
                                    wp[0]+initial_position.x,
                                    wp[1]+initial_position.y,
                                    wp[2]+initial_position.z))

            pv = plane_traj_msg.velocities[i]
            wv = self.PlaneToBaseDir(pv.x,pv.y)
            V = np.append(V, [wv], axis=0)
            velocities.append(Vector3(wv[0],wv[1],wv[2]))

        print "############### Recieved plane_traj_msg ##################"
        print "len(times): %d" % len(plane_traj_msg.times)
        print "len(positions): %d" % len(positions)
        print "len(velocities): %d" % len(velocities)
        print "shapes:"
        print T.shape
        print P.shape
        print V.shape
        print "################ T #################"
        print T
        print "################ P #################"
        print P
        print "################ V #################"
        print V
        print "##########################################################"

        self.joint_action_server(plane_traj_msg.times, positions, velocities) 


if __name__ == '__main__':
    ct = controller()
