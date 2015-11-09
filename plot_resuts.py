#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import sys
import os
import glob
# newest = max(glob.iglob('*.csv'), key=os.path.getctime)
# print newest
# files = filter(os.path.isfile, glob.glob('*.csv'))
# files.sort(key=lambda x: os.path.getmtime(x))
# print files[:3]
import tkinter as tk
from tkinter import filedialog

root = tk.Tk()
root.withdraw()
file_path = filedialog.askopenfilename()

# for a_file in sys.argv[1:]:
#     A = np.loadtxt(a_file)
#     plt.plot(A[:,0],A[:,1],"r")
#     plt.plot(A[:,0],A[:,2],"g")
#     plt.plot(A[:,0],A[:,3],"b")
# plt.show()
