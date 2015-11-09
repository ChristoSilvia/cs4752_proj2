#!/usr/bin/env python
import numpy as np
#from scipy.spatial import KDTree
#from scipy.spatial import distance
import scipy
from scipy import spatial
from scipy.spatial import distance
from scipy.spatial.distance import euclidean
from scipy.spatial import KDTree 

import rospy
from cs4752_proj2.srv import *
import baxter_interface
from baxter_interface import *
from std_msgs.msg import String

from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *
from copy import deepcopy
import random
from collision_checker.srv import CheckCollision



joint_limits = np.array([[-2.461, 0.890],
						[-2.147, 1.047],
						[-3.028, 3.028],
						[-0.052, 2.618],
						[-3.059, 3.059],
						[-1.571, 2.094],
						[-3.059, 3.059]])

def sample_cspace():
	global joint_limits
	temp = joint_limits[:,0] + (joint_limits[:,1] - joint_limits[:,0]) * np.random.rand(7)
	r = np.zeros(7)
	for i in xrange(0,7) :
		r[i] = temp[i]
	return r#sample_2d()

def sample_2d() :
	r = np.zeros(2)
	r[0] = np.random.rand(1)*5
	r[1] = np.random.rand(1)*5
	return r

class Buffered_KD_Tree :
	def __init__(self, qinit, k) :
		self.buffer=[qinit]
		self.buffer_count = 1
		self.buffer_size = k

		self.kdtree_data = None
		self.kdtree = None
		#self.kdtree_data[:] = qinit
		#self.kdtree = KDTree(self.kdtree_data)

	def insert(self, p) :
		#print "INSERTING"
		#print p.shape
		#print self.buffer_count
		if self.buffer_count == self.buffer_size :
			
			new_to_tree = np.vstack(self.buffer)
			#print "ADDING TO KD TREE"
			#print new_to_tree.shape
			if self.kdtree_data == None :

				self.kdtree_data = new_to_tree
			else :
				#print self.kdtree_data.shape
				self.kdtree_data = numpy.concatenate((self.kdtree_data, new_to_tree), axis=0)
			


			self.kdtree = KDTree(self.kdtree_data)
			self.buffer = []
			self.buffer_count = 0
			#self.buffer_size = self.buffer_size * 2

			#self.buffer_size = self.buffer_size * 2

		self.buffer.append(p)
		self.buffer_count += 1

		

	def Nearest_Neighbor(self, p) :

		closest_dist = 100000000
		closest_point = None
		for point in self.buffer :
			#this_dist = np.distance(point, p)
			this_dist = scipy.spatial.distance.euclidean(point,p)
			if this_dist < closest_dist :
				closest_buffer_dist = this_dist
				closest_point = point

		if self.kdtree != None :
			
			kd_distance, qnearI = self.kdtree.query(p)
			#print "GOT PASSSEEEEEEEEDDDDDDD QUEEEERRRRRYYYYYY"
			if kd_distance < closest_buffer_dist :
				return  self.kdtree_data[qnearI,:]

		return closest_point


#need to repair the looking_for TODO
def path_from(path, node) :
	return_path = []
	return_path.append(node)

	ind = len(path)-1
	
	while not np.array_equal((path[ind])[1], node) :
		ind = ind - 1
	looking_for = (path[ind])[0]

	for i in xrange( ind, -1, -1) :
		if looking_for == None :
			break

		if np.array_equal(looking_for, (path[i])[1]) :
			return_path.append((path[i])[1])
			looking_for = (path[i])[0]


	return return_path

#attempts a path simplification by checking a straight line path between two random points, n times
def simplify_path(path, n) :
	

	for i in xrange(0, n) :
		p1 = random.randint(0,len(path)-1)
		p2 = random.randint(0,len(path)-1)
		while p1 == p2 and len(path) > 2:
			p2 = random.uniform(0,len(path)-1)
		if Line_Collision_Check(path[p1], path[p2]) :
			#delete every segment before and after
			first = p1
			second = p2
			if first > second :
				first = p2
				second = p1
			new_path=[]
			for f in xrange(0,first) :
				new_path.append(path[f])
			for s in xrange(second, len(path)) :
				new_path.append(path[s])
			path = new_path

	return path

def PointLerp(p1, p2, delta) :
	dist = np.linalg.norm(p2 - p1)
	if dist <= delta :
		return p2
	return p1 + ((p2-p1)/dist * delta)

def loginfo(message):
    rospy.loginfo("*****************RRT: {0}".format(message))


class rrt() :
	def __init__(self) :
		rospy.init_node('rrt')
		baxter_interface.RobotEnable(CHECK_VERSION).enable()
		loginfo("Initialized rrt Node")

		self.collision_checker = rospy.ServiceProxy('/check_collision', CheckCollision)
		loginfo("Initialized /check collision")
		rospy.wait_for_service('/check_collision')
		loginfo("Finished waiting for /check collision")

		self.limb = 'left'
		self.hand_pose = None

		rospy.sleep(10)

		#will endlessly take its current position and try to move to a random position
		#while avoiding all obstacles
		while True :
			loginfo("Starting New Round of RRT Testing")
			arm = Limb(self.limb)
			angle_dict = arm.joint_angles()
			joints = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
			qi = np.zeros(7)
			for i in xrange(0,7) :
				qi[i] = angle_dict['left_'+joints[i]]
			
			loginfo("STARTING TO SAMPLE QF")
			qf = sample_cspace()
			while not self.Check_Point(qf) :
				loginfo("INVALID QF TRYING NEW SAMPLE")
				qf = sample_cspace()

			path = self.RRT_Connect_Planner(qi, qf, 5000)
			if path :
				path = simplify_path(path, len(path)/2)
				self.MoveAlongPath(path)
			else :
				print "FAILED TO FIND PATH"

		rospy.spin()

	def MoveAlongPath(self, path) :
		arm = Limb(self.limb)
		joints = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
		for p in path :
			this_p = {}
			for i in xrange(0,7) :
				this_p[joints[i]] = p[i]
			arm.set_joint_positions(this_p)

	def Check_Point(self, p) :
		
		try :
			arm = String('left')
			res = self.collision_checker(arm, list(p))
			return res.collision
		except rospy.ServiceException, e:
			loginfo("FAILED IN CHECK POINT")
			print "Service call failed: %s"%e
			return False

	def Check_Line(self, p1, p2) :
		if not self.Check_Point(p2) :
			return False

		p = p1
		while not np.array_equal(p, p2) :
			if not self.Check_Point(p) :
				return False

			#point lerp is less efficient than possible because i am finding the vector between them 
			#during each call	
			p = PointLerp(p, p2, .007) #roughly 6 degrees of rotation total
		return True

	#given an initial and final array of joints, it will find a path to there
	#avoiding any singularities and obstacles
	def RRT_Connect_Planner(self, qinit, qgoal,k) :

		Ta = Buffered_KD_Tree(qinit, 20)
		path_a = []
		path_a.append((None,qinit))

		Tb = Buffered_KD_Tree(qgoal, 20)
		path_b = []
		path_b.append((None,qgoal))

		delta = .003 #delta step to random point

		for i in xrange(0,k) : #might change to while
			#print "NEW ITERATION"
			qrand = sample_cspace()

			#print "QRAND"
			#print qrand
			#print "-----"
			
			#qrand is a valid point, now let's find the nearest neighbor,qnear
			qnear = None
			if i%2 == 1: #work from end
				qnear = Tb.Nearest_Neighbor(qrand)
			else : #work from beginning
				qnear = Ta.Nearest_Neighbor(qrand)

			#print "QNEAR"
			#print qnear
			#print "-----"

			#then take a step toward it to make qnew
			#to_vec = np.linalg.norm(qrand - qnear)
			#qnew = qnear + ((qrand-qnear)/to_vec * delta)
			
			qnew = PointLerp(qnear, qrand, delta)
			#print "QNEW:"
			#print qnew
			#print "-----"
			#do collision check between qnew and qnear
			if self.Check_Line(qnear,qnew) :
				
				#valid so add to tree

				if i%2 == 1 : #add to end
					Tb.insert(qnew)
					path_b.append((qnear,qnew))
					nearest_solution = Ta.Nearest_Neighbor(qnew)
					#print "NEAREST SOLUTION"
					#print nearest_solution
					#print "-----"
					
					end_distance = scipy.spatial.distance.euclidean(qnew, nearest_solution)
					print "at %d Distance to Solution"%i
					print end_distance
					#check if qnear is close enough to opposite tree
					if self.Check_Line(nearest_solution,qnew) :
						#the trees can reach each other to create the path
						atohere = path_from(path_a, nearest_solution).reverse()
						heretob = path_from(path_b, qnew)
						if atohere == [] or atohere == None:
							return heretob
						return atohere.append(heretob)
					
				#do opposite as above for this case
				else :
					Ta.insert(qnew)
					path_a.append((qnear,qnew))
					nearest_solution = Tb.Nearest_Neighbor(qnew)
					
					end_distance = scipy.spatial.distance.euclidean(qnew, nearest_solution)
					print "Distance to Solution"
					print end_distance
					if self.Check_Line(nearest_solution,qnew) :
						atohere = path_from(path_a, qnew).reverse()
						heretob = path_from(path_b,nearest_solution)
						if atohere == [] or atohere == None:
							return heretob
						return atohere.append(heretob)

		print "Failed to find path in k steps"
		return []



def test() :
	qi = sample_cspace()
	print "Looking for a path between: "
	print qi
	qf = sample_cspace()
	print qf

	path = RRT_Connect_Planner(qi, qf, 10000)
	print path
	pathx = []
	pathy = []
	for i in xrange(0, len(path)) :
		pathx.append((path[i])[0])
		pathy.append((path[i])[1])

	import matplotlib.pyplot as plt
	plt.plot(pathx, pathy, 'ro')
	plt.axis([0, 10, 0, 10])
	plt.show()

	if path != [] :
		print "Simplifying path"
		path = simplify_path(path, 1000)
		print path

if __name__ == '__main__':
	try: 
		rrt()
	except rospy.ROSInterruptException:
		pass
	#test()
