#!/usr/bin/env python
import numpy as np
import rospy
from cs4752_proj2.srv import *

from std_msgs.msg import String
from geometry_msgs.msg import Vector3


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

		rospy.wait_for_service("/move_end_effector_trajectory")
		self.joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)

		self.mouseX = 0
		self.mouseY = 0
		self.delMouseX = 0
		self.delMouseY = 0

		self.listLimit = 30

		self.velocityFilterLength = 5
		self.VelocityAverage = []

		self.times = []
		self.velocities = []
		self.positions = []

		self.index = 0

		self.scale = .001
		self.speed = .03 #in meters/second
		
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
		self.index = 0
		
		self.joint_action_server(self.times, self.velocities, self.positions)
		self.velocities = []
		self.times = []
		self.positions = []

	def getAverageVelocity(self) :
		mysum = [0,0]
		for v in self.VelocityAverage :
			mysum[0] += v[0]
			mysum[1] += v[1]

		return (mysum[0]/self.velocityFilterLength, mysum[1]/self.velocityFilterLength)
	


	def MouseMotion(self, event) :
		self.canvas.create_rectangle(event.x+3, event.y+3, event.x, event.y, outline="#fb0", fill="#fb0")
		self.delMouseX = event.x - self.mouseX
		self.delMouseY = event.y - self.mouseY
		self.mouseX = event.x
		self.mouseY = event.y

		

		self.VelocityAverage.append((self.delMouseX, self.delMouseY))
		if len(self.VelocityAverage) > self.velocityFilterLength :
			self.VelocityAverage.pop(0)

			self.times.append(rospy.Time.now().to_sec())

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
			
			self.index = self.index + 1

			if self.index >= self.listLimit :
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

	def OnMouseUp(self, event) :
		if self.index > 0 :
			sendLiveFeed()
		

	def quit(self) :
		self.root.quit()



if __name__ == '__main__':
	m = MouseDraw()
	