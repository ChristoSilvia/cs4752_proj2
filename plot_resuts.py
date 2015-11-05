#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import sys

A = np.loadtxt(sys.argv[1])
plt.plot(A[:,0],A[:,1])
plt.plot(A[:,0],A[:,2])
plt.plot(A[:,0],A[:,3])
plt.show()
