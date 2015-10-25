#!/usr/bin/env python
import rospy
import scipy as sp
import numpy as np
import pylab as plt
from geometry_msgs.msg import *
from cs4752_proj2.msg import Path

curr_pt = [0,0]

def loginfo(logstring):
	rospy.loginfo("Bezier: {0}".format(logstring))

def bezier_quadratic(A, B, C, t):
	"""A B and C being 2-tuples, t being a float between 0 and 1"""
	x = (1 - t)**2 * A[0] + 2 * t * (1 - t) * B[0] + t**2 * C[0]
	y = (1 - t)**2 * A[1] + 2 * t * (1 - t) * B[1] + t**2 * C[1]
	return x, y

def bezier_cubic(A, B, C, D, t):
	"""A B C and D being 2-tuples, t being a float between 0 and 1"""
	x = (1-t)**3*A[0] + 3*(1-t)**2*t*B[0] + 3*(1-t)*t**2*C[0] + t**3*D[0]
	y = (1-t)**3*A[1] + 3*(1-t)**2*t*B[1] + 3*(1-t)*t**2*C[1] + t**3*D[1]
	return x, y

def linear(A, B, t):
	x = (1-t)*A[0] + t*B[0]
	y = (1-t)*A[1] + t*B[1]
	return x, y

def pathCb(path):
	global curr_pt
	global fig
	global ax
	print "Got Path!"
	print path

	#Setup the parameterisation
	t = sp.linspace(0,1,100)
	p3 = [0,0]

	if (path.type == "R"):
		print "Reset!"
		ax.cla()
		# ax = fig.add_subplot(111)
		# ax.hold(True)
		return

	if (path.type == "C"):
		p0 = curr_pt
		p1 = [path.x1,path.y1]
		p2 = [path.x2,path.y2]
		p3 = [path.x,path.y]

		#Use the Bezier formula
		Bx, By = bezier_cubic(p0, p1, p2, p3, t)

		#Plot the Bezier curve
		ax.plot(Bx, By, "k")

	elif (path.type == "L"):
		p0 = curr_pt
		p3 = [path.x,path.y]
		Bx, By = linear(p0, p3, t)
		ax.plot(Bx, By, "k")

	elif (path.type == "M"):
		p0 = curr_pt
		p3 = [path.x,path.y]

	elif (path.type == "Z"):
		p0 = curr_pt

	#Add those to the axes
	ax.plot(p0[0],p0[1], "ob")
	ax.plot(p3[0],p3[1], "or")
	curr_pt = p3
	fig.canvas.draw()

def bezier():
	rospy.init_node("bezier")
	loginfo("Initialized node Bezier")


	#Generate the figure
	global fig
	global ax
	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.hold(True)

	rospy.Subscriber("/cmd_path", Path, pathCb, queue_size=10000)	
	
	#Save it
	# plt.savefig("totally-awesome-bezier.png")
	plt.show()
	#Bosch.
	# rospy.spin()

if __name__ == "__main__":
	bezier()
