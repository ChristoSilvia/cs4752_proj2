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
meters_per_second = .03/1
pts_per_meter = 1/.05
meters_per_unit = .1/66.6
min_pts = 5
time_between = .1
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

def bezier_quadratic_length(A, B, C, ti=0.0, tf=1.0):
	def dx(t): return 2*(1 - t) * (B[0]-A[0]) + 2*t * (C[0]-B[0])
	def dy(t): return 2*(1 - t) * (B[1]-A[1]) + 2*t * (C[1]-B[1])
	def ds(s): return np.sqrt(dx(s)**2+dy(s)**2)
	length, err = quad(ds, ti, tf)
	return length

def bezier_cubic(A, B, C, D, t):
	"""A B C and D being 2-tuples, t being a float between 0 and 1"""
	x = (1-t)**3*A[0] + 3*(1-t)**2*t*B[0] + 3*(1-t)*t**2*C[0] + t**3*D[0]
	y = (1-t)**3*A[1] + 3*(1-t)**2*t*B[1] + 3*(1-t)*t**2*C[1] + t**3*D[1]
	return x, y

def bezier_cubic_dt(A, B, C, D, t):
	dx = 3*(1-t)**2*(B[0]-A[0]) + 6*(1-t)*t*(C[0]-B[0]) + 3*t**2*(D[0]-C[0])
	dy = 3*(1-t)**2*(B[1]-A[1]) + 6*(1-t)*t*(C[1]-B[1]) + 3*t**2*(D[1]-C[1])
	return dx, dy

def bezier_cubic_length(A, B, C, D, ti=0.0, tf=1.0):
	def dx(t): return 3*(1-t)**2*(B[0]-A[0]) + 6*(1-t)*t*(C[0]-B[0]) + 3*t**2*(D[0]-C[0])
	def dy(t): return 3*(1-t)**2*(B[1]-A[1]) + 6*(1-t)*t*(C[1]-B[1]) + 3*t**2*(D[1]-C[1])
	def ds(s): return np.sqrt(dx(s)**2+dy(s)**2)
	length, err = quad(ds, ti, tf)
	return length

def linear(A, B, t):
	x = (1-t)*A[0] + t*B[0]
	y = (1-t)*A[1] + t*B[1]
	return x, y

def linear_dt(A, B, t):
	dx = [B[0] - A[0] for i in t]
	dy = [B[1] - A[1] for i in t]
	return dx, dy

def pathCb(path):
	global curr_pos, curr_vel, time, fig, ax, time_per_unit, meters_per_second, time_between

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
		# set speed
		meters_per_second = path.x
		return

	if (path.type == "C"):
		# draw cubic bezier curve
		p0 = curr_pos
		p1 = [path.x1,path.y1]
		p2 = [path.x2,path.y2]
		p3 = [path.x,path.y]

		#Setup the parameterisation
		seg_length = bezier_cubic_length(p0, p1, p2, p3)
		num_points = int(seg_length*meters_per_unit*pts_per_meter)
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

		#Plot the Bezier curve
		ax.plot(Bx, By, "k")

		# Add to trajectory msg
		P = np.array([Bx,By])
		V = np.array([Bdx,Bdy])

		# find the duration of the curent segment at constant velocity
		duration = seg_length*meters_per_unit/meters_per_second

		times = []

		# allow time to stop if not continuous path
		if not continuous:			
			P = np.insert(P,0,P[:,0],axis=1)
			V = np.insert(V,0,[0.0,0.0],axis=1)
			time += time_between/2.0
			times.append(time)
			time += time_between/2.0

		# find the duration of a piece of the segment at constant velocity
		seconds_per_unit = meters_per_unit/meters_per_second
		for curr_t in t:
			dt = bezier_cubic_length(p0, p1, p2, p3, ti=0.0, tf=curr_t)*seconds_per_unit
			times.append(time + dt)

		add_to_plane_traj_msg(P,V,times)
		time += duration

	elif (path.type == "Q"):
		# draw quadratic bezier curve
		p0 = curr_pos
		p1 = [path.x1,path.y1]
		p3 = [path.x,path.y]

		#Setup the parameterisation
		seg_length = bezier_quadratic_length(p0, p1, p3)
		num_points = int(seg_length*meters_per_unit*pts_per_meter)
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

		#Plot the Bezier curve
		ax.plot(Bx, By, "k")

		# Add to trajectory msg
		P = np.array([Bx,By])
		V = np.array([Bdx,Bdy])
		
		# find the duration of the curent segment at constant velocity
		duration = seg_length*meters_per_unit/meters_per_second

		times = []

		# allow time to stop if not continuous path
		if not continuous:
			P = np.insert(P,0,P[:,0],axis=1)
			V = np.insert(V,0,[0.0,0.0],axis=1)
			
			time += time_between/2.0
			times.append(time)
			time += time_between/2.0

		# find the duration of a piece of the segment at constant velocity
		seconds_per_unit = meters_per_unit/meters_per_second
		for curr_t in t:
			times.append(time + bezier_quadratic_length(p0, p1, p3, ti=0.0, tf=curr_t)*seconds_per_unit)
		
		add_to_plane_traj_msg(P,V,times)
		time += duration

	elif (path.type == "L"):
		# draw line to
		p0 = curr_pos
		p3 = [path.x,path.y]

		#Setup the parameterisation
		seg_length = np.linalg.norm(np.array(p3)-np.array(p0))
		num_points = int(seg_length*meters_per_unit*pts_per_meter)
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

		#Plot the Line
		ax.plot(Bx, By, "k")

		# Add to trajectory msg
		P = np.array([Bx,By])
		V = np.array([Bdx,Bdy])

		# find the duration of the curent segment at constant velocity
		duration = seg_length*meters_per_unit/meters_per_second

		# when continuous, will having 2 times that are the same mess up the joint_action_server?

		if not continuous:
			time += time_between
		
		t = [(x*duration)+time for x in t]
		# allow time to stop if not continuous path
		if not continuous:
			P = np.insert(P,0,P[:,0],axis=1)
			V = np.insert(V,0,[0.0,0.0],axis=1)
			time -= time_between/2.0
			t = np.insert(t,0,time)
			time += time_between/2.0

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

	tolerence = .15
	not_continuous = np.linalg.norm(old_vel) == 0.0 or 1-vel_diff > tolerence

	curr_vel = [dxf,dyf]
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

	z_offset = -.01
	for ti in t: plane_traj_msg.times.append(ti)
	for i in range(0,r):
		plane_traj_msg.positions.append( Vector3( P[i,0], P[i,1], z_offset ) )
		plane_traj_msg.velocities.append( Vector3( V[i,0], V[i,1], 0 ) )

def send_plane_traj():
	global plane_traj_pub, plane_traj_msg, time
	time = 0
	plane_traj_msg.reference_frame = "/plane_frame"

	print "################# send_plane_traj #################"
	print "plane_traj_msg:"
	print "plane_traj_msg.reference_frame: %s" % plane_traj_msg.reference_frame
	print "len(times): %d" % len(plane_traj_msg.times)
	print "len(positions): %d" % len(plane_traj_msg.positions)
	print "len(velocities): %d" % len(plane_traj_msg.velocities)
	print "duration: %f" % plane_traj_msg.times[len(plane_traj_msg.times)-1]
	print "###################################################"

	plane_traj_pub.publish(plane_traj_msg)
	plane_traj_msg = Trajectory()

# def move_pen_to(pi,pf):
# 	global curr_pos, curr_vel, time, fig, ax, time_per_unit, meters_per_second, time_between
# 	p0 = pi
# 	p3 = pf

# 	z_offset = .05
# 	# may want to make methods: move_line_to, move_cubic_to, move_quadratic_to

# 	#Setup the parameterisation
# 	seg_length = np.linalg.norm(np.array(p3)-np.array(p0))
# 	num_points = int(seg_length*meters_per_unit*pts_per_meter)
# 	if num_points < min_pts: num_points = min_pts
# 	t = sp.linspace(0,1,num_points)

# 	#Use the Linear  formula

# 	# add the position (Bx,By)
# 	Bx, By = linear(p0, p3, t)

# 	# add the velocity (Bdx,Bdy)
# 	Bdx, Bdy = linear_dt(p0, p3, t)

# 	dxi, dyi = linear_dt(p0, p3, t0)
# 	dxf, dyf = linear_dt(p0, p3, t1)

# 	# Add to trajectory msg
# 	P = np.array([Bx,By])
# 	V = np.array([Bdx,Bdy])

# 	# find the duration of the curent segment at constant velocity
# 	duration = seg_length*meters_per_unit/meters_per_second

# 	t = [(x*duration)+time for x in t]

# 	add_to_plane_traj_msg(P,V,t)
# 	time += duration
	
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
