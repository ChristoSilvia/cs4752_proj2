import numpy as np
from scipy.spatial import KDTree
from scipy.spatial import distance

import rospy
from cs4752_proj2.srv import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *




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
	return r


#use the course staff collision checks here
def Point_Collision_Check(point) :
	return True

def Line_Collision_Check(p1,p2) :
	return True


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
		print "INSERTING"
		if self.buffer_count == self.buffer_size :
			
			new_to_tree = np.hstack(self.buffer)
			if self.kdtree_data == None :
				self.kdtree_data = new_to_tree
			else :
				self.kdtree_data = numpy.hstack(self.kdtree_data, new_to_tree)


			self.kdtree = KDTree(self.kdtree_data)
			self.buffer = []
			self.buffer_count = 0

			#self.buffer_size = self.buffer_size * 2

		self.buffer.append(p)
		self.buffer_count += 1

		

	def Nearest_Neighbor(self, p) :
		

		closest_dist = 100000000
		closest_point = None
		for point in self.buffer :
			#this_dist = np.distance(point, p)
			this_dist = distance.euclidean(point,p)
			if this_dist < closest_dist :
				closest_buffer_dist = this_dist
				closest_point = point

		if self.kdtree != None :
			kd_distance, qnearI = self.kdtree.query(p)
			if kd_distance < closest_buffer_dist :
				return  self.kdtree_data[:,qnearI]

		return closest_point



def path_from(path, node) :
	return_path = []
	return_path.append(path[len(path)-1][1])
	looking_for = path[len(path)-1][0]
	for i in xrange( len(path)-1, -1, -1) :
		if looking_for == path[i][1] :
			return_path.append(path[i][1])
			looking_for == path[i][0]


	return return_path

#attempts a path simplification by checking a straight line path between two random points, n times
def simplify_path(path, n) :
	

	for i in xrange(0, n) :
		p1 = random.uniform(0,len(path)-1)
		p2 = random.uniform(0,len(path)-1)
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


def RRT_Connect_Planner(qinit, qgoal,k) :
	print sample_cspace()

	Ta = Buffered_KD_Tree(qinit, 20)
	path_a = []
	path_a.append((None,qinit))

	Tb = Buffered_KD_Tree(qgoal, 20)
	path_b = []
	path_b.append((None,qgoal))

	delta = .05 #delta step to random point

	for i in xrange(0,k) : #might change to while
		qrand = sample_cspace()
		while True : #this will make sure q rand is a valid point, might not be necessary
			if Point_Collision_Check(qrand) :
				break
			qrand = sample_cspace()

		
		#qrand is a valid point, now let's find the nearest neighbor,qnear
		qnear = None
		if i%2 : #work from end
			qnear = Tb.Nearest_Neighbor(qrand)
		else : #work from beginning
			qnear = Ta.Nearest_Neighbor(qrand)

		#then take a step toward it to make qnew
		to_vec = np.linalg.norm(qrand - qnear)
		qnew = qnear + (to_vec * delta)

		#do collision check between qnew and qnear
		if Line_Collision_Check(qnear,qnew) :

			#valid so add to tree

			if i%2 : #add to end
				Tb.insert(qnew)
				path_b.append((qnear,qnew))
				nearest_solution = Ta.Nearest_Neighbor(qnew)
				end_distance = distance.euclidean(qnew, nearest_solution)
				#check if qnear is close enough to opposite tree
				if end_distance <= delta and Line_Collision_Check(nearest_solution,qnew) :
					#the trees can reach each other to create the path
					atohere = path_from(path_a, nearest_solution).reverse()
					heretob = path_from(path_b, qnew)
					return atohere.append(heretob)
				
			#do opposite as above for this case
			else :
				Ta.insert(qnew)
				path_a.append((qnear,qnew))
				nearest_solution = Tb.Nearest_Neighbor(qnew)
				end_distance = distance.euclidean(qnew, nearest_solution)
				if end_distance <= delta and Line_Collision_Check(nearest_solution,qnew) :
					atohere = path_from(path_a,qnew).reverse()
					heretob = path_from(path_b,nearest_solution)
					return atohere.append(heretob)

	print "Failed to find path in k steps"
	return []



def test() :
	qi = sample_cspace()
	qf = sample_cspace()

	path = RRT_Connect_Planner(qi, qf, 10000)
	print path

	path = simplify_path(path, 1000)
	print path

if __name__ == '__main__':
    test()