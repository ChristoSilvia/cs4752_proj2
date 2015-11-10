import matplotlib.pyplot as plt
from scipy import misc
import numpy as np
from scipy import ndimage
import cv2
from Tkinter import *
import matplotlib.pyplot as plt


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

def CheckOffset(img, r, c) :
	if r >= 0 and c >= 0 and img.shape[0] > r and img.shape[1] > c :
		return img[r,c]
	return 0

#returns the straightest path available by a greedy approach
#subtracts the pixels taken in the path from the image
def FindPathFrom(img, r, c) :
	path = []
	path.append((r,c))
	img[r,c] = 0
	direction = 0
	PathFound = True
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
					break

			(ur, uc) = DirToSlope(upper)
			if CheckOffset(img, r+ur, c+uc) :
				r += ur
				c += uc
				img[r,c] = 0
				path.append((r,c))
				direction = upper
				PathFound = True
				break

			difference = difference + 1


	return img, path

def IterateImage(img) :
	paths = []
	for r in xrange(0, img.shape[0]) :
		for c in xrange(0, img.shape[1]) :
			if img[r,c] > 0 :
				img, path = FindPathFrom(img, r, c)
				paths.append(path)
	return paths

def draw_image(image_name, detail) :
	image = misc.imread(image_name)
	image = cv2.Canny(image,detail,detail)	
	#http://matplotlib.org/examples/animation/dynamic_image.html
	paths = IterateImage(image)
	print paths
	

	


if __name__ == '__main__':
    draw_image('baxter.jpeg', 150)

