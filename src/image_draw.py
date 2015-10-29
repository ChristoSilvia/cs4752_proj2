import matplotlib.pyplot as plt
from scipy import misc
import atexit

def draw_image(image_name) :

	

	test_image = misc.imread(image_name, True)
	
	
	edges = misc.imfilter(test_image, 'find_edges')



	#plt.imshow(test_image)
	#plt.show()
	
	plt.imshow(edges)
	plt.show(edges)

	#plt.close
	print "done"

@atexit.register
def closer():
	plt.close()

if __name__ == '__main__':
    draw_image('test_image.png')

