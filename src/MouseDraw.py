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

meters_per_pixel = .001

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
		self.listLimit = 90
		self.velocityFilterLength = 10
		self.scale = .001
		self.speed = .03 #in meters/second


		self.mouseX = 0
		self.mouseY = 0

		self.VelocityFilter = []
		self.times = []
		self.velocities = []
		self.positions = []
		self.timeDisplacement = 0


		
		
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

		print "*****************************"
		print "*****************************"
		print "*****************************"
		print traject.times
		print "*****************************"
		print traject.positions
		print "*****************************"
		print traject.velocities
		print "*****************************"
		print "*****************************"
		print "*****************************"

		self.plane_traj_pub.publish(traject)
		
		self.recycleTrajectoryMsgData()

	def getAverageVelocity(self) :
		mysum = [0,0]
		for v in self.VelocityFilter :
			mysum[0] += v[0]
			mysum[1] += v[1]

		return (mysum[0]/self.velocityFilterLength, mysum[1]/self.velocityFilterLength)
	
	#resets message data
	def recycleTrajectoryMsgData(self) :
		self.times = []
		self.velocities = []
		self.positions = []
		self.timeDisplacement = 

	#resets all 
	def resetTrajectoryData(self) :
		self.VelocityFilter = []
		self.recycleTrajectoryData()


	def MouseMotion(self, event) :
		self.canvas.create_rectangle(event.x+3, event.y+3, event.x, event.y, outline="#fb0", fill="#fb0")
		delMouseX = event.x - self.mouseX
		delMouseY = event.y - self.mouseY
		self.mouseX = event.x
		self.mouseY = event.y

		self.VelocityFilter.append([delMouseX, delMouseY])
		if len(self.VelocityFilter) > self.velocityFilterLength :

			if not self.times :
				print "RESETING TIME DISPLACEMENT"
				self.timeDisplacement = rospy.Time.now().to_sec()

			self.VelocityFilter.pop(0)

			self.times.append(rospy.Time.now().to_sec()-self.timeDisplacement)

			averageDelta = self.getAverageVelocity()
			mag = (averageDelta[0] **2 + averageDelta[1]**2)**.5
			if mag > 0 :
				velocity = Vector3( averageDelta[0]/mag * self.speed, averageDelta[1]/mag * self.speed, 0)
			else :
				velocity = Vector3(0,0,0)
			print velocity
			self.velocities.append(velocity)

			position = Vector3(self.mouseX*self.scale, self.mouseY*self.scale, 0)
			self.positions.append(position)
			

			if len(self.times) >= self.listLimit :
				self.sendLiveFeed()
			#print "root coordinates: %s/%s" % (event.x_root, event.y_root)

	def OnMouseScreenEnter(self, event) :
		print "Mouse in Screen"
		
	def OnMouseScreenExit(self, event) :
		print "Mouse Left Screen"

	def OnMouseDown(self, event) :
		print "Click"
		self.mouseX = event.x
		self.mouseY = event.y

		resetTrajectoryData()



	def OnMouseUp(self, event) :
		if self.index > 0 :
			sendLiveFeed()
		

	def quit(self) :
		self.root.quit()



if __name__ == '__main__':
	m = MouseDraw()
	