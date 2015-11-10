import matplotlib.pyplot as plt
from scipy import misc
import numpy as np
from scipy import ndimage
import cv2
from Tkinter import *
import matplotlib.pyplot as plt


def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.33, 0.34, 0.33])

def IterateImage(img) :
	for r in xrange(0, img.shape[0]) :
		for 

def draw_image(image_name, detail) :

	
	from scipy import misc

	image = misc.imread(image_name)
	#image = rgb2gray(image)

	
	image = cv2.Canny(image,detail,detail)


	
	f = plt.figure()
	ax = plt.imshow(image, cmap='Greys')
	#http://matplotlib.org/examples/animation/dynamic_image.html
	ani = animation.FuncAnimation(f, )
	plt.show()
	for r in xrange(0,image.shape[0]) :
		for c in xrange(0,image.shape[1]) :
			image[r,c] = 200
			ax.set_data(image)
			#show_obj.set_data(image)
			if r == c :
				print "ITERATING "
			draw()
			

			#f.canvas.draw()
	

	


if __name__ == '__main__':
    draw_image('baxter.jpeg', 150)

