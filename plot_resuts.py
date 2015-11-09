#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import sys

for a_file in sys.argv[1:]:
    A = np.loadtxt(a_file)
    plt.plot(A[:,0],A[:,1],"r")
    plt.plot(A[:,0],A[:,2],"g")
    plt.plot(A[:,0],A[:,3],"b")
plt.show()
