#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import sys
from Tkinter import Tk
from tkFileDialog import askopenfilename


if len(sys.argv[1:]) == 0:
	Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
	filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file
	# print(filename)
	A = np.loadtxt(filename)

	plt.plot(A[:,0],A[:,1],"r")
	plt.plot(A[:,0],A[:,2],"g")
	plt.plot(A[:,0],A[:,3],"b")
	plt.show()
else:
	for a_file in sys.argv[1:]:
		A = np.loadtxt(a_file)
		plt.plot(A[:,0],A[:,1],"r")
		plt.plot(A[:,0],A[:,2],"g")
		plt.plot(A[:,0],A[:,3],"b")
		plt.show()

	
