#!/usr/bin/env python
import rospy
import scipy as sp
from scipy.integrate import quad
import numpy as np
import pylab as plt
from geometry_msgs.msg import *
from cs4752_proj2.msg import *
from cs4752_proj2.srv import *

curr_pos = [0.0,0.0]
curr_vel = [0.0,0.0]
plane_traj_msg = Trajectory()
time = 0
time_per_unit = .2
pts_per_unit = 20
min_pts = 5
font_size_per_meter = 100.0/0.015
scale = 1

def loginfo(logstring):
	rospy.loginfo("Bezier: {0}".format(logstring))

def bezier_quadratic(A, B, C, t):
	"""A B and C being 2-tuples, t being a float between 0 and 1"""
	x = (1 - t)**2 * A[0] + 2 * t * (1 - t) * B[0] + t**2 * C[0]
	y = (1 - t)**2 * A[1] + 2 * t * (1 - t) * B[1] + t**2 * C[1]
	return x, y

def bezier_quadratic_dt(A, B, C, t):
	dx = 2*(1 - t) * (B[0]-A[0]) + 2*t * (C[0]-B[0])
	dy = 2*(1 - t) * (B[1]-A[1]) + 2*t * (C[1]-B[1])
	return dx, dy

def bezier_quadratic_length(A, B, C):
	def dx(t): return 2*(1 - t) * (B[0]-A[0]) + 2*t * (C[0]-B[0])
	def dy(t): return 2*(1 - t) * (B[1]-A[1]) + 2*t * (C[1]-B[1])
	def ds(s): return np.sqrt(dx(s)**2+dy(s)**2)
	length, err = quad(ds, 0.0, 1.0)
	return length

def bezier_quadratic_ddt(A, B, C, t):	
	ddx = [2*(C[0] - 2*B[0] + A[0]) for i in t]
	ddy = [2*(C[1] - 2*B[1] + A[1]) for i in t]
	return ddx, ddy

def bezier_cubic(A, B, C, D, t):
	"""A B C and D being 2-tuples, t being a float between 0 and 1"""
	x = (1-t)**3*A[0] + 3*(1-t)**2*t*B[0] + 3*(1-t)*t**2*C[0] + t**3*D[0]
	y = (1-t)**3*A[1] + 3*(1-t)**2*t*B[1] + 3*(1-t)*t**2*C[1] + t**3*D[1]
	return x, y

def bezier_cubic_dt(A, B, C, D, t):
	dx = 3*(1-t)**2*(B[0]-A[0]) + 6*(1-t)*t*(C[0]-B[0]) + 3*t**2*(D[0]-C[0])
	dy = 3*(1-t)**2*(B[1]-A[1]) + 6*(1-t)*t*(C[1]-B[1]) + 3*t**2*(D[1]-C[1])
	return dx, dy

def bezier_cubic_length(A, B, C, D):
	def dx(t): return 3*(1-t)**2*(B[0]-A[0]) + 6*(1-t)*t*(C[0]-B[0]) + 3*t**2*(D[0]-C[0])
	def dy(t): return 3*(1-t)**2*(B[1]-A[1]) + 6*(1-t)*t*(C[1]-B[1]) + 3*t**2*(D[1]-C[1])
	def ds(s): return np.sqrt(dx(s)**2+dy(s)**2)
	length, err = quad(ds, 0.0, 1.0)
	return length

def bezier_cubic_ddt(A, B, C, D, t):
	ddx = 6*(1-t)*(C[0] - 2*B[0] + A[0]) + 6*t*(D[0] - 2*C[0] + B[0])
	ddy = 6*(1-t)*(C[1] - 2*B[1] + A[1]) + 6*t*(D[1] - 2*C[1] + B[1])
	return ddx, ddy

def linear(A, B, t):
	x = (1-t)*A[0] + t*B[0]
	y = (1-t)*A[1] + t*B[1]
	return x, y

def linear_dt(A, B, t):
	dx = [B[0] - A[0] for i in t]
	dy = [B[1] - A[1] for i in t]
	return dx, dy

def linear_ddt(A, B, t):
	ddx = [0 for i in t]
	ddy = [0 for i in t]
	return ddx, ddy

def pathCb(path):
	global curr_pos, curr_vel, time, fig, ax, time_per_unit
	# print "Got Path!"
	# print path

	p0 = [0,0]
	p3 = [0,0]
	t0 = sp.linspace(0,0,1)
	t1 = sp.linspace(1,1,1)

	if (path.type == "R"):
		# reset graph
		time = 0
		ax.cla()
		return

	if (path.type == "S"):
		# set fontsize
		scale = (path.x/font_size_per_meter)
		# print "$$$$$$$$$$$$$$$$$$$$$$$$$$"
		# print "scale:"
		# print scale
		# print "$$$$$$$$$$$$$$$$$$$$$$$$$$"
		return

	if (path.type == "C"):
		# draw cubic bezier curve
		p0 = curr_pos
		p1 = [path.x1,path.y1]
		p2 = [path.x2,path.y2]
		p3 = [path.x,path.y]

		seg_length = bezier_cubic_length(p0, p1, p2, p3)
		# print "#################################"
		# print "seg_length"
		# print seg_length
		# print "#################################"

		#Setup the parameterisation
		# seg_length = np.linalg.norm(np.array(p0)-np.array(p1))
		# seg_length += np.linalg.norm(np.array(p1)-np.array(p2))
		# seg_length += np.linalg.norm(np.array(p2)-np.array(p3))
		num_points = int(seg_length*pts_per_unit)
		if num_points < min_pts: num_points = min_pts
		t = sp.linspace(0,1,num_points)

		#Use the Cubic Bezier formula

		# add the position (Bx,By)
		Bx, By = bezier_cubic(p0, p1, p2, p3, t)

		# add the velocity (Bdx,Bdy)
		Bdx, Bdy = bezier_cubic_dt(p0, p1, p2, p3, t)

		dxi, dyi = bezier_cubic_dt(p0, p1, p2, p3, t0)
		dxf, dyf = bezier_cubic_dt(p0, p1, p2, p3, t1)

		continuous = is_continuous(p0, dxi[0], dyi[0], dxf[0], dyf[0])


		# add the acceleration (Bddx,Bddy) ??
		# Bddx, Bddy = bezier_cubic_ddt(p0, p1, p2, p3, t)
		

		#Plot the Bezier curve
		ax.plot(Bx, By, "k")

		# Add to trajectory msg
		P = [Bx,By]
		V = [Bdx,Bdy]
		# A = [Bddx,Bddy]
		duration = seg_length*time_per_unit
		t = [(x*duration)+time for x in t]
		add_to_plane_traj_msg(P,V,t)
		time += duration

	elif (path.type == "Q"):
		# draw quadratic bezier curve
		p0 = curr_pos
		p1 = [path.x1,path.y1]
		p3 = [path.x,path.y]

		seg_length = bezier_quadratic_length(p0, p1, p3)

		#Setup the parameterisation
		# seg_length = np.linalg.norm(np.array(p0)-np.array(p1))
		# seg_length += np.linalg.norm(np.array(p1)-np.array(p3))
		num_points = int(seg_length*pts_per_unit)
		if num_points < min_pts: num_points = min_pts
		t = sp.linspace(0,1,num_points)
		
		#Use the Quadratic Bezier formula

		# add the position (Bx,By)
		Bx, By = bezier_quadratic(p0, p1, p3, t)

		# add the velocity (Bdx,Bdy)
		Bdx, Bdy = bezier_quadratic_dt(p0, p1, p3, t)

		dxi, dyi = bezier_quadratic_dt(p0, p1, p3, t0)
		dxf, dyf = bezier_quadratic_dt(p0, p1, p3, t1)

		continuous = is_continuous(p0, dxi[0], dyi[0], dxf[0], dyf[0])

		# add the acceleration (Bddx,Bddy) ??
		# Bddx, Bddy = bezier_quadratic_ddt(p0, p1, p3, t)


		#Plot the Bezier curve
		ax.plot(Bx, By, "k")

		# Add to trajectory msg
		P = [Bx,By]
		V = [Bdx,Bdy]
		# A = [Bddx,Bddy]
		duration = seg_length*time_per_unit
		t = [(x*duration)+time for x in t]
		add_to_plane_traj_msg(P,V,t)
		time += duration

	elif (path.type == "L"):
		# draw line to
		p0 = curr_pos
		p3 = [path.x,path.y]

		#Setup the parameterisation
		seg_length = np.linalg.norm(np.array(p3)-np.array(p0))
		num_points = int(seg_length*pts_per_unit)
		if num_points < min_pts: num_points = min_pts
		t = sp.linspace(0,1,num_points)

		#Use the Linear  formula

		# add the position (Bx,By)
		Bx, By = linear(p0, p3, t)

		# add the velocity (Bdx,Bdy)
		Bdx, Bdy = linear_dt(p0, p3, t)

		dxi, dyi = linear_dt(p0, p3, t0)
		dxf, dyf = linear_dt(p0, p3, t1)

		# check for continuity
		continuous = is_continuous(p0, dxi[0], dyi[0], dxf[0], dyf[0])

		# TODO:
		# normalize velocities
		# implement srv for move along this path 
			# by control points
			# at this speed

		# if 1st cmd
			# start vel 0
			# if next continuous
				# end vel non 0
			# else
				# end vel 0

		# if 2nd or greater cmd
			# if prev continuous
				# start vel non 0
			# else
				# start vel 0
			# if next continuous
				# end vel non 0
			# else
				# end vel 0

		# if last cmd
			# if prev continuous
				# start vel non 0
			# else
				# start vel 0
			# end vel 0

		# want to request starting vel, ending vel, starting pos, end pos, and const vel
		# have joint acition server interpolate times

		# spline is start pt, end pt, and 1 or 2 control points
		# or really just parametric equations in [x,y]
		# given a spline, and a constant velocity, follow the curve


		# add the acceleration (Bddx,Bddy) ??
		# Bddx, Bddy = linear_ddt(p0, p3, t)

		#Plot the Line
		ax.plot(Bx, By, "k")

		# Add to trajectory msg
		P = [Bx,By]
		V = [Bdx,Bdy]
		# A = [Bddx,Bddy]
		duration = seg_length*time_per_unit # might not be good for long straight lines
		t = [(x*duration)+time for x in t]
		add_to_plane_traj_msg(P,V,t)
		time += duration

	elif (path.type == "M"):
		# move pen to
		p0 = curr_pos
		p3 = [path.x,path.y]

		# move_pen_to(p3)

	elif (path.type == "Z"):
		# close path
		p0 = curr_pos

		send_plane_traj()

	#Add those to the axes
	ax.plot(p0[0],p0[1], "ob")
	ax.plot(p3[0],p3[1], "or")
	curr_pos = p3
	fig.canvas.draw()

def is_continuous(p0,dxi,dyi,dxf,dyf):
	global curr_vel
	
	old_vel = np.array(curr_vel)
	if np.linalg.norm(old_vel) != 0.0:
		old_vel /= np.linalg.norm(old_vel)
	new_vel = np.array([dxi,dyi])
	new_vel /= np.linalg.norm(new_vel)
	vel_diff = np.dot(old_vel,new_vel)

	# print "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"
	# print p0
	# # print p3
	# print old_vel
	# print new_vel
	# print "vel_diff"
	# print vel_diff
	# print "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"
	tolerence = .25
	not_continuous = np.linalg.norm(old_vel) == 0.0 or 1-vel_diff > tolerence
	if(not_continuous):
		print "#################################"
		print "Not continuous at {0}".format(p0)
		print "#################################"

	curr_vel = [dxf,dyf]
	return not not_continuous

def add_to_plane_traj_msg(P,V,t):
	""" P = position, V = velocitiy, A = acceleration t = time (sec)"""
	global plane_traj_msg, scale

	P = np.array(P).T*scale
	V = np.array(V).T*scale
	# A = np.array(A).T*scale
	r, c = P.shape

	# print "##################################"
	# print "type(P):"
	# print type(P)
	# print type(P)
	# print P.shape
	# print V.shape
	# print A.shape
	# print "##################################"

	for ti in t: plane_traj_msg.times.append(ti)
	for i in range(0,r):
		plane_traj_msg.positions.append( Vector3( P[i,0], P[i,1], 0 ) )
		plane_traj_msg.velocities.append( Vector3( V[i,0], V[i,1], 0 ) )
		# plane_traj_msg.accelerations.append( Vector3( A[i,0], A[i,1], 0 ) )

def send_plane_traj():
	global plane_traj_pub, plane_traj_msg, time
	time = 0
	plane_traj_msg.reference_frame = "/plane_frame"

	print "##################################"
	print "plane_traj_msg:"
	print "plane_traj_msg.reference_frame: %s" % plane_traj_msg.reference_frame
	print "len(times): %d" % len(plane_traj_msg.times)
	print "len(positions): %d" % len(plane_traj_msg.positions)
	print "len(velocities): %d" % len(plane_traj_msg.velocities)
	# print "len(accelerations): %d" % len(plane_traj_msg.accelerations)
	print "duration: %f" % plane_traj_msg.times[len(plane_traj_msg.times)-1]
	print "##################################"

	plane_traj_pub.publish(plane_traj_msg)
	plane_traj_msg = Trajectory()
	
def bezier():
	rospy.init_node("bezier")
	loginfo("Initialized node Bezier")

	global fig, ax, plane_traj_pub
	
	#Generate the figure
	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.hold(True)

	rospy.Subscriber("/cmd_path", Path, pathCb, queue_size=10000)	
	plane_traj_pub = rospy.Publisher('/plane_traj', Trajectory, queue_size=10)

	plt.show()
	# rospy.spin()

if __name__ == "__main__":
	bezier()
