#!/usr/bin/env python

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pickle
import argparse

parser = argparse.ArgumentParser("Plot the results from calibrating the magnetometer")
parser.add_argument('filename', metavar='filename', type=str, nargs='+',help='path to the file obtained from the calibrate_imu node')

args = parser.parse_args()
f = open(args.filename[0], 'rb')
offset = pickle.load(f)
L = pickle.load(f)
data = pickle.load(f)


# offset = np.matrix([[-0.0668749511242], [-0.0813733190298], [0.064335539937]])

normalized_data = (L*(data.T-offset)).T

print('Samples size:',len(data))
print('Magnetometer offset',offset)
print('Magnetometer Calibration Matrix: ',L)

###### ellipsoid fitting ######

## plot original in 2d (xy,yz,xz plane)
fig = plt.figure()
ax = fig.add_subplot(111)
marker_size = 1
scatter1 = ax.scatter(np.array(data[:,0]),np.array(data[:,1]),c='r',marker='^',s=1)
scatter2 = ax.scatter(np.array(data[:,0]),np.array(data[:,2]),c='g',marker='o',s=1)
scatter3 = ax.scatter(np.array(data[:,1]),np.array(data[:,2]),c='b',marker='s',s=1)
plt.legend((scatter1,scatter2,scatter3),
           ('X-Y Plane', 'Y-Z Plane', 'X-Z Plane'),
           scatterpoints=3,
           markerscale=2,
           loc='upper right',
           ncol=3,
           fontsize=8)

# plot the correct in 2d
offset_x = np.full(data[:,0].shape, offset[0])
offset_y = np.full(data[:,0].shape, offset[1])
offset_z = np.full(data[:,0].shape, offset[2])

fig = plt.figure()
ax = fig.add_subplot(111)
marker_size = 1
scatter4 = ax.scatter(np.array(data[:,0]-offset_x),np.array(data[:,1] - offset_y),c='r',marker='^',s=1)
scatter5 = ax.scatter(np.array(data[:,0]-offset_x),np.array(data[:,2] - offset_z),c='g',marker='o',s=1)
scatter6 = ax.scatter(np.array(data[:,1]-offset_y),np.array(data[:,2] - offset_z),c='b',marker='s',s=1)
plt.legend((scatter4,scatter5,scatter6),
           ('X-Y Plane', 'Y-Z Plane', 'X-Z Plane'),
           scatterpoints=3,
           markerscale=2,
           loc='upper right',
           ncol=3,
           fontsize=8)

## plot original in 3d(ellipsoid)
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.scatter(np.array(data[:,0]),np.array(data[:,1]),np.array(data[:,2]),c='b',marker='^',s=0.1)
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
## plot the correct in 3d (sphere)
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.scatter(np.array(normalized_data[:,0]),np.array(normalized_data[:,1]),np.array(normalized_data[:,2]),c='r',marker='o',s=0.1)
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
