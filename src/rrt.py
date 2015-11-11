#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
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
import matplotlib.pyplot as plt

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
	r[0] = np.random.rand(1)*20
	r[1] = np.random.rand(1)*20
	return r

def Collision_Test2D(p) :
	return p[0] > 6 and p[0] <14 and p[1] > 5 and p[1] < 15

class Buffered_KD_Tree :
	def __init__(self, qinit, k, dimension) :
		self.buffer=[qinit]
		self.buffer_count = 1
		self.buffer_size = k

		self.kdtree_data = None
		self.kdtree = None
		self.dimension = dimension
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
			if self.dimension != 2 :
				this_dist = scipy.spatial.distance.euclidean(point,p)
			else :
				this_dist = ((point[0]-p[0])**2 + (point[1]-p[1])**2)**.5
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

def Draw_All_Paths(path_a, path_b, ax) :
		for (parent,child) in path_a :
			if parent != None and child != None :
				ax.plot([parent[0], child[0]],[parent[1],child[1]], 'r')
		for (parent,child) in path_b :
			if parent != None and child != None :
				ax.plot([parent[0], child[0]],[parent[1],child[1]], 'g')
		print 'drawing all paths'
class rrt() :
	def __init__(self) :
		rospy.init_node('rrt')
		

		self.TWOD=1
		self.CSPACE =2

		self.mode = self.CSPACE

		self.limb = 'left'
		self.hand_pose = None

		if self.mode != self.TWOD :
			baxter_interface.RobotEnable(CHECK_VERSION).enable()
			loginfo("Initialized rrt Node")
			self.collision_checker = rospy.ServiceProxy('/check_collision', CheckCollision)
			loginfo("Initialized /check collision")
			rospy.wait_for_service('/check_collision')
			loginfo("Finished waiting for /check collision")

		else :

			fig = plt.figure()
			ax = fig.add_subplot(111)
			ax.hold(True)

			qi2 = np.array([1,1])#sample_2d()
			while Collision_Test2D(qi2) :
				qi2 = sample_2d()
			qf2 = np.array([19,19])#sample_2d()
			while Collision_Test2D(qf2) :
				qf2 = sample_2d()
			path = self.RRT_Connect_Planner(qi2,qf2,5000, ax)
			pathx = []
			pathy = []
			if path :
				print "2d path found"
				for i in xrange(0,len(path)) :
					if path[i] != None and path[i] != []:
						print path[i]
						pathx.append(path[i][0])
						pathy.append(path[i][1])
				ax = fig.add_subplot(111)
				ax.hold(True)
				ax.plot(pathx, pathy , 'b')
				print "------Simplifying Paths----------"
				path = self.simplify_path(path, len(path)/2)
				print path
				pathx = []
				pathy = []
				for point in path :
					if point != None and point != []:
						pathx.append(point[0])
						pathy.append(point[1])
				
				ax.plot(pathx, pathy , 'k')

			plt.show()
			plt.close()
			return



		#will endlessly take its current position and try to move to a random position
		#while avoiding all obstacles
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

		path = self.RRT_Connect_Planner(qi, qf, 5000, None)
		if path :
			
			print "Found Path"
			print path

			for i in xrange(0, len(path)-2) :
				if self.Check_Line(path[i], path[1+i]) :
					print "DEEEEEEEEEEEEEEEPPPPPPPPPP ISSSSSSSUUUUUUEEEEEESSSSSS"

			#self.MoveAlongPathVelocity2(path)
			print "Simplifying"
			print len(path)
			path = self.simplify_path(path, len(path)/2)
			print "--------------------------------------"
			print path
			print "--------------------------------------"
			print len(path)
			
			print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
			print qi
			print '/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/'
			print qf
			print '--------**********************--------'
			self.MoveAlongPathVelocity2(path)
			print "------------------------------"
			
		else :
			print "FAILED TO FIND PATH"

		rospy.spin()

	# def getJointAngles() :

	

	def MoveAlongPathVelocity2(self, path) :
		joint_velocity = 1
		min_distance = .05
		kp = 2.0
		arm = Limb(self.limb)
		joints = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
		for goal in path :
			if goal == None :
				print "*************NONE in RRT MoveAlongPathVelocity2"
			print "Moving To Next Node In Path"
			current_p_dict = arm.joint_angles()
			start_p = np.zeros(7)
			for i in xrange(0,7) :
				start_p[i] = current_p_dict['left_'+joints[i]]
			#if self.Check_Line(start_p, goal) :
			#	print "******************************************************"
			#	print "COLLISION IN PATH. RRT IS FLAWED"
			#	print "======================================================"
			#	return
			start_to_goal = goal - start_p
			start_vec = (start_to_goal)/(np.linalg.norm(start_to_goal))
			start_time = rospy.get_time()
			max_time = np.linalg.norm(start_to_goal) / joint_velocity
			
			while True :
				newTime = rospy.get_time()
				deltaTime = newTime - start_time
				if deltaTime >= max_time :
					deltaTime = max_time
					ideal_pos = goal
					start_to_goal = np.zeros(7)
				else :
					ideal_pos = start_vec*deltaTime*joint_velocity + start_p

				current_p_dict = arm.joint_angles()
				current_p = np.zeros(7)
				for i in xrange(0,7) :
					current_p[i] = current_p_dict['left_'+joints[i]]
				
				current_to_ideal = ideal_pos - current_p

				to_goal_vector = np.zeros(7)
				to_goal_vector =  kp*current_to_ideal + start_to_goal#*(1-deltaTime/max_time) # vector .5
				
				distance = scipy.spatial.distance.euclidean(goal, current_p)
				print 'distance to next pose :'
				print distance
				#print 'error : %f' % np.linalg.norm(current_to_ideal)
				#print current_to_ideal
				#print 'Moving toward :'
				#print to_goal_vector
				#print 'ideal_pos'
				#print ideal_pos
				#print 'current_p'
				#print current_p
				
				if distance < min_distance :
					to_goal_vector= np.zeros(7)
				#to_goal_vector = ((goal - current_p)/distance)*path_speed
				velocity_dict = {}
				for i in xrange(0,7) :
					velocity_dict['left_'+joints[i]] = to_goal_vector[i]

				arm.set_joint_velocities(velocity_dict)

				
				if distance < min_distance :
					break
				else :
					rospy.sleep(.050)

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

		if self.mode == self.TWOD :
			return Collision_Test2D(p)

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
	def RRT_Connect_Planner(self, qinit, qgoal,k, ax) :
		if self.mode == self.TWOD :
			temp_dim = 2
		else :
			temp_dim = 7
		Ta = Buffered_KD_Tree(qinit, 20, temp_dim)
		path_a = []
		path_a.append((None,qinit))

		Tb = Buffered_KD_Tree(qgoal, 20, temp_dim)
		path_b = []
		path_b.append((None,qgoal))

		delta = .2 #delta step to random point

		for i in xrange(0,k) : #might change to while
			#print "NEW ITERATION"
			if ax :
				qrand = sample_2d()
			else :
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
					if end_distance < delta and not self.Check_Line(nearest_solution,qnew) :
						#the trees can reach each other to create the path
						print path_a
						print nearest_solution
						atohere = path_from(path_a, nearest_solution)
						atohere.reverse()
						heretob = path_from(path_b, qnew)
						
						if self.mode == self.TWOD :
							Draw_All_Paths(path_a,path_b, ax)
						if atohere == [] or atohere == None:
							print "atohere"
							print "How can atohere be empty??"
							return heretob
						#atohere.append(heretob)

						return atohere+heretob
					
				#do opposite as above for this case
				else :
					Ta.insert(qnew)
					path_a.append((qnear,qnew))
					nearest_solution = Tb.Nearest_Neighbor(qnew)
					
					end_distance = scipy.spatial.distance.euclidean(qnew, nearest_solution)
					print "Distance to Solution"
					print end_distance
					if end_distance < delta and not self.Check_Line(nearest_solution,qnew) :
						print path_a
						print qnew
						atohere = path_from(path_a, qnew)
						atohere.reverse()
						heretob = path_from(path_b,nearest_solution)

						if self.mode == self.TWOD :
							Draw_All_Paths(path_a,path_b, ax)
						if atohere == [] or atohere == None:
							print "How can atohere be empty??"
							return heretob
						#atohere.append(heretob)
						return atohere+heretob

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
