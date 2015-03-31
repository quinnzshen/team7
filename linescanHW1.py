
# coding: utf-8

## EECS192 Sring 2015 Track Finding from 1D line sensor data

# In[1]:

get_ipython().magic(u'pylab')


# In[2]:

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import scipy.ndimage as ndi  # useful for 1d filtering functions
# comment the following line to have external plots
get_ipython().magic(u'matplotlib inline')


# In[3]:

# Graphing helper function
def setup_graph(title='', x_label='', y_label='', fig_size=None):
    fig = plt.figure()
    if fig_size != None:
        fig.set_size_inches(fig_size[0], fig_size[1])
    ax = fig.add_subplot(111)
    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)


# Line scan plotting function.
# 

# In[4]:

def plot_frame(linearray):
    nframes = np.size(linearray)/128
    n = range(0,129)
    print 'number of frames', nframes
    for i in range(0, nframes-1):
        setup_graph(title='$x[n]$', x_label='$n$', y_label='row'+str(i)+' $ xa[n]$', fig_size=(15,2))
        plt.subplot(1,3,1)
        _ = plt.plot(n,linearray[i,:])
        plt.subplot(1,3,2)
        _ = plt.plot(n,linearray[i+1,:])
    # plot simple difference between frames
        plt.subplot(1,3,3)
        _ = plt.plot(n,linearray[i+1,:] - linearray[i,:])
        plt.ylabel('Frame n+1 - Frame n')


# In[16]:

### inputs:
# linescans - An array of length n where each element is an array of length 128. Represents n frames of linescan data.

### outputs:
# track_center_list - A length n array of integers from 0 to 127. Represents the predicted center of the line in each frame.
# track_found_list - A length n array of booleans. Represents whether or not each frame contains a detected line.
# cross_found_list - A length n array of booleans. Represents whether or not each fram contains a crossing.

def find_track(linescans):
    n = len(linescans)
    track_center_list = n * [64]
    track_found_list = n * [True]
    cross_found_list = n * [False]
    
    # Basic: Peak detection
    for i in range(n):
        # invert it first
        list = linescans[i]
        
        # Option 1 - Frame subtraction and peak detection
        # find_peaks_cwt(vector, widths[, wavelet, ...]) Attempt to find the peaks in a 1-D array.
        # peaks = signal.find_peaks_cwt(data, np.arange(100,200))
        
        # Option 2 - Finding minima
        # argrelmin(data[, axis, order, mode])	Calculate the relative minima of data
        minima = sp.argrelmin(list)
        
        # Option 3 - Gradient detection
        # gauss_spline(x, n)	Gaussian approximation to B-spline basis function of order n.
        
        # Option 4 = Curve Fitting
        # cubic(x)	A cubic B-spline.
        # We can do curve fitting using and then find the minimum.

        #track_center_list set
        #track_found_list is set when we found the value that is lower than some threshold value in an array
        #cross_found_list is set when more than two minima are founds
        
    # so for each frame(array) we need to find where center is(0~128)
    # Difference of Gaussians approximation to the Laplacian
    # gauss_spline(x, n)	Gaussian approximation to B-spline basis function of order n
    
    return track_center_list, track_found_list, cross_found_list


# In[17]:

linea = np.genfromtxt('ee192_sp14_camera_testdata_a.csv', delimiter=",")
nframes = np.size(linea)/128
print 'number of frames', nframes

linescans = []
for i in range(0, nframes):
    line = linea[i,0:128]
    linescans.append(line)

track_center_list, track_found_list, cross_found_list = find_track(linescans)
for i, (track_center, track_found, cross_found) in enumerate(zip(track_center_list, track_found_list, cross_found_list)):
    print 'scan # %d center at %d. Track_found = %s, Cross_found = %s' %(i,track_center,track_found, cross_found)


#### Set A linescans

# In[7]:

n = range(0,129)
linea = np.genfromtxt('ee192_sp14_camera_testdata_a.csv', delimiter=",")
plot_frame(linea)
line1 = linea[0,:]
print line1


#### Set B line scans

# In[8]:

lineb = np.genfromtxt('ee192_sp14_camera_testdata_b.csv', delimiter=",")
plot_frame(lineb)
line1 = lineb[0,:]
print line1


#### Set C linescans

# In[9]:

linec = np.genfromtxt('ee192_sp14_camera_testdata_c.csv', delimiter=",")
plot_frame(linec)
line1 = linec[0,:]
print line1


# In[41]:

# clipped data - examples with exposure gain not set correctly

linec = np.genfromtxt('ee192_sp14_camera_testdata_c.csv', delimiter=",")
setup_graph(title='$x[n]$', x_label='$n$', y_label='intensity', fig_size=(15,2))
print 'linec=',linec[0,:]
print 'n=', n
#plt.plot(n,linec[0,:])
linescale = np.zeros(129)
linescale1 = np.zeros(129)
#print 'linescale=', linescale[:]
lines1 = linec[0,:]
for i in range(0,128):
    pixel = linec[0,i]
    linescale[i] = min(65535,3.0*pixel)
    linescale1[i] = max(0, 0.2*pixel - 5000)
#print 'linescale=', linescale[:]    
#plt.subplot(1,3,1)
_ = plt.plot(n,linescale)
setup_graph(title='$x[n]$', x_label='$n$', y_label='intensity', fig_size=(15,2))
#plt.subplot(1,3,2)
_ = plt.plot(n,linescale1)


# In[41]:



