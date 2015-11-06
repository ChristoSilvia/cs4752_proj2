#!/usr/bin/env python
import numpy as np
import rospy
from cs4752_proj2.srv import *

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from cs4752_proj2.msg import Trajectory


from copy import deepcopy
import random
from Tkinter import *

meters_per_pixel = .0007

def loginfo(message):
    rospy.loginfo("MouseDraw: {0}".format(message))

class MouseDraw() :
	def __init__(self) :
		rospy.init_node('MouseDraw')
		loginfo("Initialized MouseDraw Node")

		#rospy.wait_for_service("/move_end_effector_trajectory")
		#self.joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)
		self.plane_traj_pub = rospy.Publisher('/plane_traj', Trajectory, queue_size=50)

		#constants
		self.limb = 'left'
		self.listLimit = 20
		self.velocityFilterLength = 10
		self.scale = .0005
		self.speed = .03 #in meters/second
		self.TrajectoryUpdateWait = .02

		self.mouseX = 0
		self.mouseY = 0

		self.VelocityFilter = []
		self.times = []
		self.velocities = []
		self.positions = []
		self.timeDisplacement = 0
		self.lastPosition = None



		self.lastTrajectoryUpdate = 0
		
		
		
		self.root = Tk()
		self.canvas = Canvas(width=512, height=512, bg='white')
		self.canvas.pack(expand=YES, fill=BOTH) 
		self.canvas.bind("<1>", self.OnMouseDown)
		self.canvas.bind("<Enter>", self.OnMouseScreenEnter)
		self.canvas.bind("<Leave>", self.OnMouseScreenExit)
		self.canvas.bind("<B1-Motion>", self.MouseMotion)
		self.canvas.bind('c', self.Clear)
		print "drawing on canvas"
		
		self.root.mainloop()

	def Clear(self, event) :
		print "Deleting Canvas"
		self.canvas.get_tk_widget().delete("all")

	def sendLiveFeed(self) :
		traject = Trajectory()
		traject.reference_frame = self.limb
		traject.times = self.times
		traject.positions = self.positions
		traject.velocities =  self.velocities

		#print self.times

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
			lastPosition = self.positions[len(self.positions)-1]
		else :
			lastPosition = None
		self.times = []
		self.velocities = []
		self.positions = []


	#resets all 
	def resetTrajectoryData(self) :
		self.VelocityFilter = []
		self.recycleTrajectoryMsgData()
		lastPosition = None


	def MouseMotion(self, event) :
		
		delMouseX = event.x - self.mouseX
		delMouseY = event.y - self.mouseY
		self.mouseX = event.x
		self.mouseY = event.y



		#self.VelocityFilter.append([delMouseX, delMouseY])

		
		#if len(self.VelocityFilter) > self.velocityFilterLength :
		#	self.VelocityFilter.pop(0)
		if rospy.Time.now().to_sec() - self.lastTrajectoryUpdate > self.TrajectoryUpdateWait :
			self.canvas.create_rectangle(event.x+3, event.y+3, event.x, event.y, outline="#fb0", fill="#fb0")
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
			

			

			position = Vector3(self.mouseX*self.scale, self.mouseY*self.scale, 0)
			delTime = 0
			if self.positions : #lists have elements in them so you can update time and velocity accordingly
				oldpos = self.positions[len(self.positions)-1]
				distanceTraveled = ((position.x - oldpos.x) **2 + (position.y - oldpos.y)**2)**.5
				delTime = (distanceTraveled/self.speed)
				self.times.append(self.times[len(self.times)-1]+delTime)

				velocity = Vector3( (position.x - oldpos.x)/distanceTraveled * self.speed, (position.y - oldpos.y)/distanceTraveled * self.speed, 0)
				self.velocities.append(velocity)
			else : #nothing, so just add zero
				if self.lastPosition != None :
					dt = ((position.x - self.lastPosition.x) **2 + (position.y - self.lastPosition.y)**2)**.5
					velocity = Vector3((position.x-self.lastPosition.x)/dt *self.speed, (position.y-self.lastPosition.y)/dt * self.speed, 0)
				else :
					velocity = Vector3( 0, 0, 0)
				self.velocities.append(velocity)
				self.times.append(0)
			
			self.positions.append(position)
			
			if len(self.times) >= self.listLimit :
				self.sendLiveFeed()
			self.lastTrajectoryUpdate = rospy.Time.now().to_sec()
			#print "root coordinates: %s/%s" % (event.x_root, event.y_root)

	def OnMouseScreenEnter(self, event) :
		print "Mouse in Screen"
		
	def OnMouseScreenExit(self, event) :
		print "Mouse Left Screen"

	def OnMouseDown(self, event) :
		print "Click"
		self.mouseX = event.x
		self.mouseY = event.y

		self.resetTrajectoryData()



	def OnMouseUp(self, event) :
		if self.index > 0 :
			sendLiveFeed()
		

	def quit(self) :
		self.root.quit()



if __name__ == '__main__':
	m = MouseDraw()
	