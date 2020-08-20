'''
    Visualize filtered lane data (smoothing done with moving average filter)
'''

import numpy as np
import matplotlib.pyplot as plt

from math import atan

data = np.loadtxt('whole_lane_data.dat')

avg = 30 #change this to modfiy how many points we average over
for z in range(0, len(data)-avg-1):

    avg_x = 0
    avg_y = 0

    for j in range(0, avg):
        pt = data[z+j]
        avg_x = avg_x + pt[0]
        avg_y = avg_y + pt[1]



    plt.scatter(avg_x/avg, avg_y/avg)
plt.show()
