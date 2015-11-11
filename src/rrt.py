#!/usr/bin/env python
import numpy as np
#from scipy.spatial import KDTree
#from scipy.spatial import distance
import time
import scipy
from scipy import spatial
from scipy.spatial import distance
from scipy.spatial.distance import euclidean
from scipy.spatial import KDTree 
from numpy import linalg

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
			while self.Check_Point(qi) : #true that there is an obstacle
				print "QI is Not valid"
				qi = np.zeros(7)
				angle_dict = arm.joint_angles()
				for i in xrange(0,7) :
					qi[i] = angle_dict['left_'+joints[i]]
			
			loginfo("STARTING TO SAMPLE QF")
			qf = sample_cspace()
			while self.Check_Point(qf) :
				loginfo("INVALID QF TRYING NEW SAMPLE")
				qf = sample_cspace()

			path = self.RRT_Connect_Planner(qi, qf, 5000)
			if path :
				
				print "Found Path"
				print path
				self.MoveAlongPathVelocity2(path)
				print "Simplifying"
				path = self.simplify_path(path, len(path)/2)
				print "--------------------------------------"
				print path
				print "Found Path Moving Along Path!"
				
				print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
				print qi
				print '/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/'
				print qf
				print '--------**********************--------'
				
				print "------------------------------"
				
			else :
				print "FAILED TO FIND PATH"

		rospy.spin()

	# def getJointAngles() :

	

	def MoveAlongPathVelocity2(self, path) :
		joint_velocity = .2
		min_distance = .10
		kp = 0.0
		arm = Limb(self.limb)
		joints = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
		for goal in path :
			print "Moving To Next Node In Path"
			current_p_dict = arm.joint_angles()
			start_p = np.zeros(7)
			for i in xrange(0,7) :
				start_p[i] = current_p_dict['left_'+joints[i]]
			start_to_goal = goal - start_p
			start_vec = (start_to_goal)/(np.linalg.norm(start_to_goal))
			start_time = time.time()
			
			while True :
				newTime = time.time()
				deltaTime = newTime - start_time
				ideal_pos = start_vec*deltaTime + start_p
				current_p_dict = arm.joint_angles()
				current_p = np.zeros(7)
				for i in xrange(0,7) :
					current_p[i] = current_p_dict['left_'+joints[i]]
				distance = scipy.spatial.distance.euclidean(goal, current_p)
				current_to_ideal = ideal_pos - current_p
				# current_to_ideal *= -1
				correction_dire = current_to_ideal/np.linalg.norm(current_to_ideal)
				to_goal_vector =  correction_dire*kp + start_vec # vector .5
				to_goal_norm = (to_goal_vector/np.linalg.norm(to_goal_vector)) * joint_velocity
				print 'distance to next pose :'
				print distance
				print 'error : %f' % np.linalg.norm(current_to_ideal)
				print current_to_ideal
				print 'Moving toward :'
				print to_goal_norm
				
				if distance < min_distance :
					to_goal_norm = np.zeros(7)

				#to_goal_vector = ((goal - current_p)/distance)*path_speed
				velocity_dict = {}
				for i in xrange(0,7) :
					velocity_dict['left_'+joints[i]] = to_goal_norm[i]

				arm.set_joint_velocities(velocity_dict)

				rospy.sleep(.05)
				if distance < min_distance :
					break

	def MoveAlongPathVelocity(self, path) :
		path_speed = .8
		arm = Limb(self.limb)
		joints = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
		for goal in path :
			print "Moving To Next Node In Path"
			while True :

				current_p_dict = arm.joint_angles()
				current_p = np.zeros(7)
				for i in xrange(0,7) :
					current_p[i] = current_p_dict['left_'+joints[i]]
				distance = scipy.spatial.distance.euclidean(goal, current_p)
				print distance
				print 'distance'
				if distance < .1 :
					break

				to_goal_vector = ((goal - current_p)/distance)*path_speed
				velocity_dict = {}
				for i in xrange(0,7) :
					velocity_dict['left_'+joints[i]] = to_goal_vector[i]

				arm.set_joint_velocities(velocity_dict)

				rospy.sleep(.07)
			
				
				
	def MoveAlongPath(self, path) :
		arm = Limb(self.limb)
		joints = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
		for p in path :
			this_p = {}
			for i in xrange(0,7) :
				this_p['left_'+joints[i]] = p[i]
			arm.set_joint_positions(this_p)
			not_there_yet = True
			while not_there_yet :
				print "waiting til next move"
				rospy.sleep(.1)
				new_p = arm.joint_angles()
				tran_pos = np.zeros(7)
				for xi in xrange(0,7) :
					tran_pos[xi] = new_p['left_'+joints[xi]]
				not_there_yet = scipy.spatial.distance.euclidean(p, tran_pos) > .001
				print scipy.spatial.distance.euclidean(p, tran_pos)

	def Check_Point(self, p) :

		try :
			self.collision_checker = rospy.ServiceProxy('/check_collision', CheckCollision)
			rospy.wait_for_service('/check_collision')
			arm = String("left")
			tep = list(p)
			res = self.collision_checker(arm, tep)
			return res.collision
		except rospy.ServiceException, e:
			loginfo("FAILED IN CHECK POINT")
			print "Service call failed: %s"%e
			return True

	def Check_Line(self, p1, p2) :
		if self.Check_Point(p2) :
			return True

		p = p1
		while not np.array_equal(p, p2) :
			if self.Check_Point(p) :
				return True

			#point lerp is less efficient than possible because i am finding the vector between them 
			#during each call	
			p = PointLerp(p, p2, .07) #roughly 6 degrees of rotation total
		return False

	#attempts a path simplification by checking a straight line path between two random points, n times
	def simplify_path(self, path, n) :
		

		for i in xrange(0, n) :
			if len(path) < 4 :
				return path
			p1 = random.randint(1,len(path)-2)
			p2 = random.randint(1,len(path)-2)
			while p1 == p2 :
				p2 = random.randint(1,len(path)-2)
			if not self.Check_Line(path[p1], path[p2]) :
				#delete every segment before and after
				first = p1
				second = p2
				if first > second :
					first = p2
					second = p1
				new_path=[]
				for f in xrange(0,first+1) :
					new_path.append(path[f])
				for s in xrange(second, len(path)) :
					new_path.append(path[s])
				path = new_path

		return path

	#given an initial and final array of joints, it will find a path to there
	#avoiding any singularities and obstacles
	def RRT_Connect_Planner(self, qinit, qgoal,k) :

		Ta = Buffered_KD_Tree(qinit, 20)
		path_a = []
		path_a.append((None,qinit))

		Tb = Buffered_KD_Tree(qgoal, 20)
		path_b = []
		path_b.append((None,qgoal))

		delta = .2 #delta step to random point

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
			if not self.Check_Line(qnear,qnew) :
				
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
					if not self.Check_Line(nearest_solution,qnew) :
						#the trees can reach each other to create the path
						atohere = path_from(path_a, nearest_solution).reverse()
						heretob = path_from(path_b, qnew)
						if atohere == [] or atohere == None:
							print "How can atohere be empty??"
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
					if not self.Check_Line(nearest_solution,qnew) :
						atohere = path_from(path_a, qnew).reverse()
						heretob = path_from(path_b,nearest_solution)
						if atohere == [] or atohere == None:
							print "How can atohere be empty??"
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
