#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
from picamera import PiCamera
#sudo apt-get install python3-picamera
#sudo apt-get install python3-pip
#sudo python3 -m pip install opencv-python
#sudo python3 -m pip install scipy
#import numpy as np
from numpy import pi as np_pi
from numpy import load as np_load
from numpy import nditer as np_nditer
from numpy import size as np_size
from numpy import where as np_where
from numpy import nanmean as np_nanmean
from numpy import shape as np_shape
from numpy import reshape as np_reshape
from numpy import exp as np_exp
from numpy import cos as np_cos
from numpy import sin as np_sin
from numpy import empty as np_empty
from numpy import full as np_full
from numpy import linspace as np_linspace
from numpy import arange as np_arange
from numpy import multiply as np_multiply
from numpy import matmul as np_matmul
from numpy import round as np_round
from numpy import max as np_max
from numpy import min as np_min
from numpy import maximum as np_maximum
from numpy import argmax as np_argmax
from numpy import convolve as np_convolve
from numpy import sum as np_sum
from numpy import uint8 as np_uint8
from numpy import nan as np_nan
from numpy import isnan as np_isnan
from numpy import float32 as np_float32

#stepper setup
microsteps = 8
step = 1.8/microsteps #deg
steppause = 0.005
rotation_dist = 9 # has to be int !!!
rotation_steps = int(rotation_dist/step) # has to be int !!!

#setup measurements
camera_laser_dist = 12.25
camera_plattform_dist = 23
DEG2RAD = np_pi/180

#gaussian kernel for image processing
kernelsize = 15
sigma = 3
array = (np_arange(kernelsize)-kernelsize/2+0.5)
gaussian_kernel = np_exp(-array**2/(2*sigma**2))#*1/np_sqrt(2*np_pi*sigma**2)
gaussian_kernel /= np_sum(gaussian_kernel)
#gaussian_reshaped = np_reshape( gaussian_kernel , [-1,1] )
#gaussian2D = np_matmul( gaussian_reshaped , gaussian_reshaped.T )
#del gaussian_reshaped

### GPIO setup
GPIO.setmode(GPIO.BCM)

#laser
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
#turn laser off
GPIO.output(5,0)
GPIO.output(6,0)

#button
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.wait_for_edge(13, GPIO.FALLING)

#stepper
GPIO.setup(22, GPIO.OUT) #dir
GPIO.setup(23, GPIO.OUT) #step
GPIO.setup(24, GPIO.OUT) #enable
GPIO.output(22,0) #clockwise
GPIO.output(23,0) #pull down step
GPIO.output(24,1) #disable

#camera setup
#res = (2592, 1944)
res = (2592, 1936)
camera = PiCamera(resolution=res, framerate=2)
camera.iso = 200 #800
camera.shutter_speed = 10000 #100000
camera.exposure_mode = 'off'
camera.awb_gains = 1
camera.awb_mode = 'off'

#image and distorsions
IMG = np_empty((res[0], res[1], 3), dtype=np_uint8)
#from cv2 import remap
#mapx = np_load('/home/localuser/3Dscanner/scan_images/mapx.npy')
#mapy = np_load('/home/localuser/3Dscanner/scan_images/mapy.npy')

### results
datapoints = []

### process images

def clip0(img):
    return np_maximum(img,0)

def scale(img):
    maxi = np_max(img)
    mini = np_min(img)
    return (img-mini)/(maxi-mini)

#gauss = lambda x,a,x0,sigma: a*np_exp(-(x-x0)**2/(2*sigma**2))
#from scipy.optimize import curve_fit
#def find_maxima_gauss(redIMG):
#    dim = np_shape(redIMG)
#    maxima = np_full(dim[0],np_nan, dtype=np_float32)
#    x = np_arange(dim[1])
#    for k in range(dim[0]):
#        try:
#            if np_max(redIMG[k,:])<1:
#                continue
#            popt,pcov = curve_fit(gauss,x,redIMG[k,:],p0=[250,dim[1]/2,20])
#            maxima[k] = popt[1]
#        except:
#            pass
#    return maxima

def gaussian_filter(IMG):
    for ind in range(np_size(IMG,0)):
        IMG[ind,:] = np_convolve( IMG[ind,:] , gaussian_kernel ,'same')
    for ind in range(np_size(IMG,1)):
        IMG[:,ind] = np_convolve( IMG[:,ind] , gaussian_kernel ,'same')
    return IMG

def find_maxima_peak(redIMG):
    dim = np_shape(redIMG)
    maxima = np_full(dim[0],np_nan, dtype=np_float32)
    for iii in range(0,dim[0]):
        index = np_argmax(redIMG[iii,:])
        if index != 0:
            maxima[iii] = index
    return maxima

def processing(redIMG):
    #gaussian blurr
    redIMG = gaussian_filter(redIMG)
    #threshold image
    threshold = 0.1#*np_max(redIMG)
    thresholded = np_multiply(redIMG,redIMG>threshold)
    #find maxima with gaussian function (fit)
#    maxima = find_maxima_gauss(thresholded)
    maxima = find_maxima_peak(thresholded)
    #remove outliners
    dim = np_shape(redIMG)
    window = dim[1]//10 #max horizontal step
    w=20 #window size
    for p in range(len(maxima)-w):
        av = np_nanmean(maxima[p:p+w]) #average position inside window
        if (maxima[p+w//2] - av) > window:
            maxima[p+w//2] = np_nan
    return maxima

### save datapoints to .ply
def save_point_cloud(datapoints):
    with open('scan_result.ply', 'w') as file:
        #write header
        file.write('ply\n')
        file.write('format ascii 1.0\n')
        file.write('element vertex '+str(len(datapoints))+'\n')
        file.write('property float x\n')
        file.write('property float y\n')
        file.write('property float z\n')
        file.write('end_header\n')
        #write 3D points
        for point in datapoints:
            file.write(str(point)+'\n')
    return None

### stepper rotation
def rotate():
    for i in range(rotation_steps):
        GPIO.output(23,1)
        sleep(steppause)
        GPIO.output(23,0)
        sleep(steppause)
    return None

#wait for calibration, turn lasers on
GPIO.wait_for_edge(13, GPIO.FALLING)
sleep(2)
GPIO.output(5,1)
GPIO.output(6,1)

#confirm successful calibration, take calibration image
GPIO.wait_for_edge(13, GPIO.FALLING)
sleep(2)
#capture image
##camera.capture('/home/localuser/3Dscanner/scan_images/calibration.jpg')
#pointcloud generation
camera.capture(IMG, "rgb")
redIMG = scale(clip0(IMG[:,:,0] - (IMG[:,:,1]+IMG[:,:,2])/2))
#redIMG = cv2.remap(redIMG,mapx,mapy,cv2.INTER_CUBIC)
maxima = processing(redIMG)
zero_shift = np_nanmean( maxima )
#x = maxima - zero_shift
#y = np_where(~np_isnan(maxima))
#z = np_round( x * camera_plattform_dist/camera_laser_dist ,2)
#for ind in np_nditer(y):
#    datapoints.append(str(x[ind])+' '+str(ind)+' '+str(z[ind]))
GPIO.output(5,0)
GPIO.output(6,0)

#remove calibration screen, start scanning
GPIO.wait_for_edge(13, GPIO.FALLING)
sleep(2)

#rotate and take pictures
GPIO.output(24,0) #enable motor
for deg in range(0,360,rotation_dist):
    
    #rotate stepper
    rotate()
    
    #first image (left laser)
    GPIO.output(5,1)
    #capture image
    ##camera.capture('/home/localuser/3Dscanner/scan_images/img_l_%03d.jpg' % deg)
    #pointcloud generation
    camera.capture(IMG, "rgb")
    redIMG = scale(clip0(IMG[:,:,0] - (IMG[:,:,1]+IMG[:,:,2])/2))
#    redIMG = cv2.remap(redIMG,mapx,mapy,cv2.INTER_CUBIC)
    maxima = processing(redIMG)
    #zero_shift = np_nanmean( maxima )
    x = maxima - zero_shift
    y = np_where(~np_isnan(maxima))
    z = np_round( x * camera_plattform_dist/camera_laser_dist ,2)
    if np_size(y)>0:
        for ind in np_nditer(y):
            xx = x[ind]*np_cos(deg*DEG2RAD)
            yy = ind
            zz = z[ind]*np_sin(deg*DEG2RAD)
            datapoints.append(str(xx)+' '+str(yy)+' '+str(zz))
    GPIO.output(5,0)
    
    #second image (right laser)
    GPIO.output(6,1)
    #capture image
    ##camera.capture('/home/localuser/3Dscanner/scan_images/img_r_%03d.jpg' % deg)
    #pointcloud generation
    camera.capture(IMG, "rgb")
    redIMG = scale(clip0(IMG[:,:,0] - (IMG[:,:,1]+IMG[:,:,2])/2))
#    redIMG = cv2.remap(redIMG,mapx,mapy,cv2.INTER_CUBIC)
    maxima = processing(redIMG)
    #zero_shift = np_nanmean( maxima )
    x = maxima - zero_shift
    y = np_where(~np_isnan(maxima))
    z = - np_round( x * camera_plattform_dist/camera_laser_dist ,2)
    if np_size(y)>0:
        for ind in np_nditer(y):
            xx = x[ind]*np_cos(deg*DEG2RAD)
            yy = ind
            zz = z[ind]*np_sin(deg*DEG2RAD)
            datapoints.append(str(xx)+' '+str(yy)+' '+str(zz))
    GPIO.output(6,0)
GPIO.output(24,1) #disable motor
save_point_cloud(datapoints)

#restart the script itself
#import subprocess
#subprocess.call(['sudo /etc/init.d/full_scan.sh start'], shell=False)