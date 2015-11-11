#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import numpy as np
import rospy
from cs4752_proj2.srv import *
from config import *
from tkFileDialog import askopenfilename

from std_msgs.msg import String
from geometry_msgs.msg import *
from cs4752_proj2.msg import Trajectory


import matplotlib.pyplot as plt
from scipy import misc
import cv2
from Tkinter import *
from copy import deepcopy
import random

def loginfo(message):
    rospy.loginfo("MouseDraw: {0}".format(message))

def DirToSlope(dir) :
		if dir == 0 :
			return (1,0)
		elif dir == 1 :
			return (1,1)
		elif dir == 2 :
			return (0,1)
		elif dir == 3 :
			return (-1,1)
		elif dir == 4 :
			return (-1,0)
		elif dir == 5 :
			return (-1,-1)
		elif dir == 6 :
			return (0,-1)
		else :
			return (1,-1)

def isInBounds(img,r,c) :
	return r >= 0 and c >= 0 and img.shape[0] > r and img.shape[1] > c

def CheckOffset(img, r, c) :
	if isInBounds(img,r,c) :
		return img[r,c]
	return 0

def DeletePerps(img, r, c, dire, stride) :
	upper = (dire + 2)%8
	(ur, uc) = DirToSlope(upper)
	lower = ((dire - 2)+8)%8
	(lr,lc) = DirToSlope(lower)
	for i in xrange(0,stride) :
		if isInBounds(img, r+ur*i,c+uc*i) :
			img[r+ur*i,c+uc*i] = 0
		if isInBounds(img, r+lr*i,c+lc*i) :
			img[r+lr*i,c+lc*i] = 0
	return img

#returns the straightest path available by a greedy approach
#subtracts the pixels taken in the path from the image
def FindPathFrom(img, r, c) :
	path = []
	path.append((r,c))
	img[r,c] = 0
	direction = 0
	PathFound = True
	delete_stride = 3
	while (PathFound) :
		difference = 0
		PathFound = False
		while difference < 5 :
			upper = (direction + difference)%8
			lower = ((direction - difference)+8)%8
			if upper != lower :
				(lr, lc) = DirToSlope(lower)
				if CheckOffset(img, r+lr, c+lc) :
					r += lr
					c += lc
					path.append((r,c))
					img[r,c] = 0
					direction = lower
					PathFound = True
					#canvas.create_rectangle(r+2, c+2, r, c, outline="#fb0", fill="#fb0")
					img = DeletePerps(img, r,c,direction,delete_stride)
					break

			(ur, uc) = DirToSlope(upper)
			if CheckOffset(img, r+ur, c+uc) :
				r += ur
				c += uc
				img[r,c] = 0
				path.append((r,c))
				direction = upper
				PathFound = True
				
				img = DeletePerps(img, r,c,direction, delete_stride)
				break

			difference = difference + 1
	if len(path) > 5 :
		return img, path
	return img, []

def IterateImage(img) :
	paths = []
	for r in xrange(0, img.shape[0]) :
		for c in xrange(0, img.shape[1]) :
			if img[r,c] > 0 :
				img, path = FindPathFrom(img, r, c)
				if path :
					paths.append(path)
	return paths

def draw_image(image_name, detail) :
	image = misc.imread(image_name)
	(rows,cols,channels) = image.shape
	image = cv2.Canny(image,detail,detail)
	
	# img_width = int(.3/self.scale)
	# img_width = 500
	# image = cv2.resize(image,(img_width,rows*img_width/cols))
	# M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
	# image = cv2.warpAffine(image,M,(cols*img_width/rows,img_width))

	# plt.imshow(image)
	# # plt.imshow(image2)
	# plt.hold()
	# plt.show()

	paths = IterateImage(image)
	print paths
	return paths

class MouseDraw() :
	def __init__(self) :
		rospy.init_node('MouseDraw')
		loginfo("Initialized MouseDraw Node")


		self.plane_traj_pub = rospy.Publisher('/plane_traj', Trajectory, queue_size=10)
		self.plane_traj_srv = createServiceProxy('move_plane_traj', JointAction, "left")

		#rospy.wait_for_service('/')

		self.move_robot_plane = createServiceProxy('move_robot_plane', MoveRobot, "")

		loginfo("Initialized service proxy for /move_robot")

		#constants
		self.limb = 'left'
		self.listLimit = 20
		self.velocityFilterLength = 10
		self.scale = .0007 #in meters/pixel
		self.speed = .03 #in meters/second
		self.ZDelta = .04
		self.TrajectoryUpdateWait = .02

		self.mouseX = 0
		self.mouseY = 0

		self.VelocityFilter = []
		self.times = []
		self.velocities = []
		self.positions = []
		self.timeDisplacement = 0
		self.lastPosition = None
		self.zDist = 0
		self.zVel = 0
		self.lastzDist = 0



		self.lastTrajectoryUpdate = 0
		


		self.root = Tk()
		self.canvas = Canvas(width=512, height=512, bg='white')
		self.canvas.pack(expand=YES, fill=BOTH) 
		self.canvas.bind("<1>", self.OnMouseDown)
		#self.canvas.bind("<Enter>", self.OnMouseScreenEnter)
		#self.canvas.bind("<Leave>", self.OnMouseScreenExit)
		self.canvas.bind("<B1-Motion>", self.MouseMotion)
		self.root.bind_all('<4>', self.on_mousewheelDown,  add='+')
		self.root.bind_all('<5>', self.on_mousewheelUp,  add='+')
		self.canvas.bind('c', self.Clear)


		filename = askopenfilename(initialdir="~/ros_ws/src/cs4752_proj2/img/") 
		image_path = draw_image(filename, 150)
		print len(image_path)
		self.sendImagePath(image_path)

		print "drawing on canvas"
		
		self.root.mainloop()

	def Clear(self, event) :
		print "Deleting Canvas"
		self.canvas.get_tk_widget().delete("all")

	def sendImagePath(self, paths) :
		oldpos = Vector3(0,0,0)
		for path in paths :
			loginfo("Starting New Line")
			(x,y) =path[0]
			self.MoveToScreenPosition(x,y,self.zDist)


			traject = Trajectory()
			traject.reference_frame = self.limb
			traject.times =[]
			traject.positions = []
			traject.velocities = []
			i = 0
			sumtime = 0
			for pixels in path :
				if i % 5 == 0 :
					self.canvas.create_rectangle(pixels[0], pixels[1], pixels[0]+2, pixels[1]+2, outline="#fb0", fill="#fb0")
					
					new_p = Vector3(pixels[0] *self.scale, pixels[1] *self.scale, self.zDist)
					new_v = Vector3(0,0,0)
					new_t = 0
					if i != 0 :
						dx = new_p.x - oldpos.x
						dy = new_p.y - oldpos.y
						dz = new_p.z - oldpos.z
						dist = (dx**2+dy**2+dz**2)**.5
						new_v = Vector3((dx)/dist*self.speed, (dy)/dist*self.speed, dz/dist*self.speed)
						new_t = traject.times[len(traject.times)-1] + dist/self.speed
					sumtime += new_t
					traject.positions.append(new_p)
					traject.velocities.append(new_v)
					traject.times.append(new_t)
					oldpos = new_p
				i += 1
			self.canvas.update()

			# self.plane_traj_pub.publish(traject)
			# rospy.sleep(sumtime)
			self.plane_traj_srv(traject.times,traject.positions,traject.velocities)



	def sendLiveFeed(self) :
		traject = Trajectory()
		traject.reference_frame = self.limb
		traject.times = self.times
		traject.positions = self.positions
		traject.velocities =  self.velocities

		print 'sending trajectory'

		self.plane_traj_pub.publish(traject)
		
		self.recycleTrajectoryMsgData()

	def getAverageVelocity(self) :
		actualLength = len(self.VelocityFilter) 
		print "AVERAGE DETAILS::"
		print actualLength
		print self.velocityFilterLength
		if self.velocityFilterLength != actualLength:
			print "ERROR Velocity filter length is not equal to constant"

		mysum = [0,0]
		for v in self.VelocityFilter :
			mysum[0] += v[0]
			mysum[1] += v[1]

		return (mysum[0]/actualLength, mysum[1]/actualLength)
	
	#resets message data
	def recycleTrajectoryMsgData(self) :
		if self.positions :
			self.lastPosition = self.positions[len(self.positions)-1]
		else :
			self.lastPosition = None
		self.times = []
		self.velocities = []
		self.positions = []


	#resets all 
	def resetTrajectoryData(self) :
		self.VelocityFilter = []
		self.recycleTrajectoryMsgData()
		self.lastPosition = None


	def MouseMotion(self, event) :
		
		self.delMouseX = event.x - self.mouseX
		self.delMouseY = event.y - self.mouseY
		self.mouseX = event.x
		self.mouseY = event.y
		if self.applyMotion() :
			self.canvas.create_rectangle(event.x+3, event.y+3, event.x, event.y, outline="#fb0", fill="#fb0")


	def applyMotion(self) :
		#self.VelocityFilter.append([delMouseX, delMouseY])

		
		#if len(self.VelocityFilter) > self.velocityFilterLength :
		#	self.VelocityFilter.pop(0)
		if rospy.Time.now().to_sec() - self.lastTrajectoryUpdate > self.TrajectoryUpdateWait :
		
			#if not self.times :
			#	print "RESETING TIME DISPLACEMENT"
			#	self.timeDisplacement = rospy.Time.now().to_sec()

			

			#self.times.append(rospy.Time.now().to_sec()-self.timeDisplacement)

			#averageDelta = self.getAverageVelocity()
			#mag = (averageDelta[0] **2 + averageDelta[1]**2)**.5
			#if mag > 0 :
			#	velocity = Vector3( averageDelta[0]/mag * self.speed, averageDelta[1]/mag * self.speed, 0)
			#else :
			#	velocity = Vector3(0,0,0)
			

			

			position = Vector3(self.mouseX*self.scale, self.mouseY*self.scale, self.zDist)
			delTime = 0
			velocity = Vector3(0,0,0)
			if self.positions : #lists have elements in them so you can update time and velocity accordingly
				oldpos = self.positions[len(self.positions)-1]
				distanceTraveled = ((position.x - oldpos.x) **2 + (position.y - oldpos.y)**2 + (self.zDist - self.lastzDist)**2)**.5
				if distanceTraveled == 0 :
					self.times.append(self.TrajectoryUpdateWait)
				else :
					delTime = (distanceTraveled/self.speed)
					self.times.append(self.times[len(self.times)-1]+delTime)
					velocity = Vector3( (position.x - oldpos.x)/distanceTraveled * self.speed, (position.y - oldpos.y)/distanceTraveled * self.speed, self.zVel)
				self.velocities.append(velocity)
			else : #nothing, so just add zero
				if self.lastPosition != None :
					dt = ((position.x - self.lastPosition.x) **2 + (position.y - self.lastPosition.y)**2+ (self.zDist - self.lastzDist)**2)**.5
					velocity = Vector3((position.x-self.lastPosition.x)/dt *self.speed, (position.y-self.lastPosition.y)/dt * self.speed, self.zVel)
				else :
					velocity = Vector3( 0, 0, self.zVel)
				self.times.append(0)
				self.velocities.append(velocity)
				
			

			self.positions.append(position)
			self.zVel = 0
			self.lastzDist = self.zDist
			if len(self.times) >= self.listLimit :
				self.sendLiveFeed()
			self.lastTrajectoryUpdate = rospy.Time.now().to_sec()
			self.delMouseX = 0
			self.delMouseY = 0

			return True 

		return False
	
	def MoveToScreenPosition(self,x,y,z) :
		newpose = Pose()
		newpose.position = Vector3()
		newpose.position.x = x *self.scale
		newpose.position.y = y *self.scale
		newpose.position.z = z
		newpose.orientation = Quaternion(0,0,0,1)
		success = self.move_robot_plane(MOVE_TO_POS, "left", newpose)
		return success

	def OnMouseDown(self, event) :
		self.mouseX = event.x
		self.mouseY = event.y

		self.MoveToScreenPosition(self.mouseX, self.mouseY, self.zDist)


		self.resetTrajectoryData()



	def on_mousewheelUp(self, event) :
		self.zVel += self.ZDelta
		print self.zVel
		if self.zVel > 0 :
			self.zDist += 1.5 * (self.zVel**2)
		else :
			self.zDist -= 1.5 * (self.zVel**2)
		self.applyMotion()


	def on_mousewheelDown(self, event) :
		print "MOUSE WHEEL DOWN"
		self.zVel -= self.ZDelta
		print self.zVel
		if self.zVel > 0 :
			self.zDist += 1.5 * (self.zVel**2)
		else :
			self.zDist -= 1.5 * (self.zVel**2)
		print self.zDist
		self.applyMotion()



	def OnMouseUp(self, event) :
		if self.index > 0 :
			sendLiveFeed()
		

	def quit(self) :
		self.root.quit()

	
		




if __name__ == '__main__':
	m = MouseDraw()
	