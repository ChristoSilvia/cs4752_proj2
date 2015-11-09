#!/usr/bin/env python
import rospy
import scipy as sp
from scipy.integrate import quad
import numpy as np
import pylab as plt
from geometry_msgs.msg import *
from cs4752_proj2.msg import *
from cs4752_proj2.srv import *
from copy import deepcopy

curr_pos = [0.0,0.0,0.0]
curr_vel = [0.0,0.0,0.0]
plane_traj_msg = Trajectory()
time = 0
meters_per_second = .03/1
pts_per_meter = 1/.01
meters_per_unit = .1/66.6
min_pts = 5
time_between = .1
scale = 1
z_offset = .020

def loginfo(logstring):
	rospy.loginfo("Bezier: {0}".format(logstring))

def bezier_quadratic(A, B, C, t):
	"""A B and C being 2-tuples, t being a float between 0 and 1"""
	x = (1 - t)**2 * A[0] + 2 * t * (1 - t) * B[0] + t**2 * C[0]
	y = (1 - t)**2 * A[1] + 2 * t * (1 - t) * B[1] + t**2 * C[1]
	z = (1 - t)**2 * A[2] + 2 * t * (1 - t) * B[2] + t**2 * C[2]
	return x, y, z

def bezier_quadratic_dt(A, B, C, t):
	dx = 2*(1 - t) * (B[0]-A[0]) + 2*t * (C[0]-B[0])
	dy = 2*(1 - t) * (B[1]-A[1]) + 2*t * (C[1]-B[1])
	dz = 2*(1 - t) * (B[2]-A[2]) + 2*t * (C[2]-B[2])
	return dx, dy, dz

def bezier_quadratic_length(A, B, C, ti=0.0, tf=1.0):
	def dx(t): return 2*(1 - t) * (B[0]-A[0]) + 2*t * (C[0]-B[0])
	def dy(t): return 2*(1 - t) * (B[1]-A[1]) + 2*t * (C[1]-B[1])
	def dz(t): return 2*(1 - t) * (B[2]-A[2]) + 2*t * (C[2]-B[2])
	def ds(s): return np.sqrt(dx(s)**2+dy(s)**2+dz(s)**2)
	length, err = quad(ds, ti, tf)
	return length

def bezier_cubic(A, B, C, D, t):
	"""A B C and D being 2-tuples, t being a float between 0 and 1"""
	x = (1-t)**3*A[0] + 3*(1-t)**2*t*B[0] + 3*(1-t)*t**2*C[0] + t**3*D[0]
	y = (1-t)**3*A[1] + 3*(1-t)**2*t*B[1] + 3*(1-t)*t**2*C[1] + t**3*D[1]
	z = (1-t)**3*A[2] + 3*(1-t)**2*t*B[2] + 3*(1-t)*t**2*C[2] + t**3*D[2]
	return x, y, z

def bezier_cubic_dt(A, B, C, D, t):
	dx = 3*(1-t)**2*(B[0]-A[0]) + 6*(1-t)*t*(C[0]-B[0]) + 3*t**2*(D[0]-C[0])
	dy = 3*(1-t)**2*(B[1]-A[1]) + 6*(1-t)*t*(C[1]-B[1]) + 3*t**2*(D[1]-C[1])
	dz = 3*(1-t)**2*(B[2]-A[2]) + 6*(1-t)*t*(C[2]-B[2]) + 3*t**2*(D[2]-C[2])
	return dx, dy, dz

def bezier_cubic_length(A, B, C, D, ti=0.0, tf=1.0):
	def dx(t): return 3*(1-t)**2*(B[0]-A[0]) + 6*(1-t)*t*(C[0]-B[0]) + 3*t**2*(D[0]-C[0])
	def dy(t): return 3*(1-t)**2*(B[1]-A[1]) + 6*(1-t)*t*(C[1]-B[1]) + 3*t**2*(D[1]-C[1])
	def dz(t): return 3*(1-t)**2*(B[2]-A[2]) + 6*(1-t)*t*(C[2]-B[2]) + 3*t**2*(D[2]-C[2])
	def ds(s): return np.sqrt(dx(s)**2+dy(s)**2+dz(s)**2)
	length, err = quad(ds, ti, tf)
	return length

def linear(A, B, t):
	x = (1-t)*A[0] + t*B[0]
	y = (1-t)*A[1] + t*B[1]
	z = (1-t)*A[2] + t*B[2]
	return x, y, z

def linear_dt(A, B, t):
	dx = [B[0] - A[0] for i in t]
	dy = [B[1] - A[1] for i in t]
	dz = [B[2] - A[2] for i in t]
	return dx, dy, dz

def draw_cubic_bezier(p0,p1,p2,p3):
	global curr_pos, curr_vel, time, fig, ax, time_per_unit, meters_per_second, time_between
	t0 = sp.linspace(0,0,1)
	t1 = sp.linspace(1,1,1)

	# draw cubic bezier curve
	
	#Setup the parameterisation
	seg_length = bezier_cubic_length(p0, p1, p2, p3)
	num_points = int(seg_length*meters_per_unit*pts_per_meter)
	if num_points < min_pts: num_points = min_pts
	T_max = 1.0
	t = sp.linspace(0,1,num_points)
	Tau = (t/T_max)**2 * (4.0 - 4.0*(t/T_max) + (t/T_max)**2)
	Tauprime = 2*(t/(T_max**2))*(4.0 - 4.0*(t/T_max) + (t/T_max)**2) + (t/T_max)**2 * (-4.0/T_max + 2.0*(t/(T_max**2)))

	#Use the Cubic Bezier formula

	# add the position (Bx,By)
	Bx, By, Bz = bezier_cubic(p0, p1, p2, p3, Tau)

	# add the velocity (Bdx,Bdy)
	Bdx, Bdy, Bdz = bezier_cubic_dt(p0, p1, p2, p3, Tau)
	Bdx *= Tauprime
	Bdy *= Tauprime
	Bdz *= Tauprime

	dxi, dyi, dzi = bezier_cubic_dt(p0, p1, p2, p3, t0)
	dxf, dyf, dzf = bezier_cubic_dt(p0, p1, p2, p3, t1)

	continuous = is_continuous(p0, [dxi[0], dyi[0], dzi[0]], [dxf[0], dyf[0], dzf[0]])

	#Plot the Bezier curve
	# ax.plot(Bx, By, "k")
	

	# Add to trajectory msg
	P = np.array([Bx,By,Bz])
	V = np.array([Bdx,Bdy,Bdz])

	# find the duration of the curent segment at constant velocity
	duration = seg_length*meters_per_unit/meters_per_second

	times = []

	# allow time to stop if not continuous path
	if not continuous:			
		P = np.insert(P,0,P[:,0],axis=1)
		V = np.insert(V,0,[0.0,0.0,0.0],axis=1)
		time += time_between/2.0
		times.append(time)
		time += time_between/2.0

	# find the duration of a piece of the segment at constant velocity
	seconds_per_unit = meters_per_unit/meters_per_second
	for curr_t in t:
		dt = bezier_cubic_length(p0, p1, p2, p3, ti=0.0, tf=curr_t)*seconds_per_unit
		times.append(time + dt)

	ax.plot(times, Bx, "r")
	ax.plot(times, By, "g")
	ax.plot(times, Bz, "b")

	add_to_plane_traj_msg(P,V,times)
	time += duration

def draw_quadratic_bezier(p0,p1,p3):
	global curr_pos, curr_vel, time, fig, ax, time_per_unit, meters_per_second, time_between
	t0 = sp.linspace(0,0,1)
	t1 = sp.linspace(1,1,1)

	# draw quadratic bezier curve

	#Setup the parameterisation
	seg_length = bezier_quadratic_length(p0, p1, p3)
	num_points = int(seg_length*meters_per_unit*pts_per_meter)
	if num_points < min_pts: num_points = min_pts
	t = sp.linspace(0,1,num_points)
	T_max = 1.0
	Tau = (t/T_max)**2 * (4.0 - 4.0*(t/T_max) + (t/T_max)**2)
	Tauprime = 2*(t/(T_max**2))*(4.0 - 4.0*(t/T_max) + (t/T_max)**2) + (t/T_max)**2 * (-4.0/T_max + 2.0*(t/(T_max**2)))
	
	#Use the Quadratic Bezier formula

	# add the position (Bx,By)
	Bx, By, Bz = bezier_quadratic(p0, p1, p3, Tau)

	# add the velocity (Bdx,Bdy)
	Bdx, Bdy, Bdz = bezier_quadratic_dt(p0, p1, p3, Tau)
	Bdx *= Tauprime
	Bdy *= Tauprime
	Bdz *= Tauprime

	dxi, dyi, dzi = bezier_quadratic_dt(p0, p1, p3, t0)
	dxf, dyf, dzf = bezier_quadratic_dt(p0, p1, p3, t1)

	continuous = is_continuous(p0, [dxi[0], dyi[0], dzi[0]], [dxf[0], dyf[0], dzf[0]])

	# Plot the Bezier curve
	# ax.plot(Bx, By, "k")

	# Add to trajectory msg
	P = np.array([Bx,By,Bz])
	V = np.array([Bdx,Bdy,Bdz])
	
	# find the duration of the curent segment at constant velocity
	duration = seg_length*meters_per_unit/meters_per_second

	times = []

	# allow time to stop if not continuous path
	if not continuous:
		P = np.insert(P,0,P[:,0],axis=1)
		V = np.insert(V,0,[0.0,0.0,0.0],axis=1)
		
		time += time_between/2.0
		times.append(time)
		time += time_between/2.0

	# find the duration of a piece of the segment at constant velocity
	seconds_per_unit = meters_per_unit/meters_per_second
	for curr_t in t:
		times.append(time + bezier_quadratic_length(p0, p1, p3, ti=0.0, tf=curr_t)*seconds_per_unit)
	
	ax.plot(times, Bx, "r")
	ax.plot(times, By, "g")
	ax.plot(times, Bz, "b")
	
	add_to_plane_traj_msg(P,V,times)
	time += duration

def draw_line(p0,p3,plot=True):
	global curr_pos, curr_vel, time, fig, ax, time_per_unit, meters_per_second, time_between
	t0 = sp.linspace(0,0,1)
	t1 = sp.linspace(1,1,1)

	# draw line to
	
	#Setup the parameterisation
	seg_length = np.linalg.norm(np.array(p3)-np.array(p0))
	num_points = int(seg_length*meters_per_unit*pts_per_meter)
	if num_points < min_pts: num_points = min_pts
	t = sp.linspace(0,1,num_points)
	T_max = 1.0
	Tau = (t/T_max)**2 * (4.0 - 4.0*(t/T_max) + (t/T_max)**2)
	Tauprime = 2*(t/(T_max**2))*(4.0 - 4.0*(t/T_max) + (t/T_max)**2) + (t/T_max)**2 * (-4.0/T_max + 2.0*(t/(T_max**2)))

	#Use the Linear  formula

	# add the position (Bx,By)
	Bx, By, Bz = linear(p0, p3, Tau)

	# add the velocity (Bdx,Bdy)
	Bdx, Bdy, Bdz = linear_dt(p0, p3, Tau)
	Bdx *= Tauprime
	Bdy *= Tauprime
	Bdz *= Tauprime

	dxi, dyi, dzi = linear_dt(p0, p3, t0)
	dxf, dyf, dzf = linear_dt(p0, p3, t1)

	# check for continuity
	continuous = is_continuous(p0, [dxi[0], dyi[0], dzi[0]], [dxf[0], dyf[0], dzf[0]])

	#Plot the Line
	# if plot:
	# 	ax.plot(Bx, By, "k")

	# Add to trajectory msg
	P = np.array([Bx,By,Bz])
	V = np.array([Bdx,Bdy,Bdz])

	# find the duration of the curent segment at constant velocity
	duration = seg_length*meters_per_unit/meters_per_second

	# when continuous, will having 2 times that are the same mess up the joint_action_server?

	if not continuous:
		time += time_between
	
	t = [(x*duration)+time for x in t]
	# allow time to stop if not continuous path
	if not continuous:
		P = np.insert(P,0,P[:,0],axis=1)
		V = np.insert(V,0,[0.0,0.0,0.0],axis=1)
		time -= time_between/2.0
		t = np.insert(t,0,time)
		time += time_between/2.0

	if plot:
		ax.plot(t, Bx, "r")
		ax.plot(t, By, "g")
		ax.plot(t, Bz, "b")
	

	add_to_plane_traj_msg(P,V,t)
	time += duration

def move_pen(p0,p3):
	global z_offset, curr_pos

	z_offset_unit = z_offset/meters_per_unit
	
	p1 = deepcopy(p0)
	p1[2] = z_offset_unit
	p2 = deepcopy(p3)
	p2[2] = z_offset_unit

	p4 = deepcopy(p3)
	p4[2] = -z_offset_unit/2.

	draw_line(p0,p1,plot=False)
	curr_pos = p1
	draw_line(p1,p2,plot=False)
	curr_pos = p2
	draw_line(p2,p4,plot=False)

def pathCb(path):
	global curr_pos, curr_vel, time, fig, ax, time_per_unit, meters_per_second, time_between

	p0 = [0,0,0]
	p3 = [0,0,0]

	if (path.type == "R"):
		# reset graph
		time = 0
		ax.cla()
		return

	if (path.type == "S"):
		# set speed
		meters_per_second = path.x
		return

	if (path.type == "C"):
		p0 = curr_pos
		p1 = [path.x1,path.y1,0]
		p2 = [path.x2,path.y2,0]
		p3 = [path.x,path.y,0]
		draw_cubic_bezier(p0,p1,p2,p3)	
		
	elif (path.type == "Q"):
		p0 = curr_pos
		p1 = [path.x1,path.y1,0]
		p3 = [path.x,path.y,0]
		draw_quadratic_bezier(p0,p1,p3)		

	elif (path.type == "L"):
		p0 = curr_pos
		p3 = [path.x,path.y,0]
		draw_line(p0,p3)

	elif (path.type == "M"):
		send_plane_traj()
		# move pen to
		p0 = curr_pos
		p3 = [path.x,path.y,0]

		print "#####################################"
		print "move pen to"
		print p3
		print "#####################################"

		move_pen(p0,p3)

		send_plane_traj()

	elif (path.type == "Z"):
		# close path
		p0 = curr_pos

		# p1 = deepcopy(p0)
		# z_offset_unit = .05/meters_per_unit
		# p1[2] = z_offset_unit
		# draw_line(p0,p1,plot=False)
		# curr_pos = p1
		
		send_plane_traj()

	#Add those to the axes
	# ax.plot(p0[0],p0[1], "ob")
	# ax.plot(p3[0],p3[1], "or")
	curr_pos = p3
	fig.canvas.draw()

def is_continuous(p0,vi,vf):
	global curr_vel
	
	old_vel = np.array(curr_vel)
	if np.linalg.norm(old_vel) != 0.0:
		old_vel /= np.linalg.norm(old_vel)
	new_vel = np.array(vi)
	new_vel /= np.linalg.norm(new_vel)
	vel_diff = np.dot(old_vel,new_vel)

	tolerence = .1
	not_continuous = np.linalg.norm(old_vel) == 0.0 or 1-vel_diff > tolerence

	curr_vel = vf
	return not not_continuous

def add_to_plane_traj_msg(P,V,t):
	""" P = position, V = velocitiy, A = acceleration t = time (sec)"""
	global plane_traj_msg, meters_per_unit, meters_per_second

	# scale position
	P = P.T*meters_per_unit

	# normalize velocity then scale to desired speed
	V = V.T
	V /= np.linalg.norm(V, axis=-1)[:, np.newaxis]
	V[np.isnan(V) | np.isinf(V)]=0
	V *= meters_per_second

	r, c = P.shape

	global z_offset

	for ti in t: plane_traj_msg.times.append(ti)
	for i in range(0,r):
		plane_traj_msg.positions.append( Vector3( P[i,0], P[i,1], P[i,2]-z_offset ) )
		plane_traj_msg.velocities.append( Vector3( V[i,0], V[i,1], V[i,2] ) )

def send_plane_traj():
	global plane_traj_pub, plane_traj_msg, time
	time = 0
	plane_traj_msg.reference_frame = "/plane_frame"

	if len(plane_traj_msg.times) > 0:
		# print "################# send_plane_traj #################"
		# print "plane_traj_msg:"
		# print "plane_traj_msg.reference_frame: %s" % plane_traj_msg.reference_frame
		# print "len(times): %d" % len(plane_traj_msg.times)
		# print "len(positions): %d" % len(plane_traj_msg.positions)
		# print "len(velocities): %d" % len(plane_traj_msg.velocities)
		# print "duration: %f" % plane_traj_msg.times[len(plane_traj_msg.times)-1]
		# print "###################################################"
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
