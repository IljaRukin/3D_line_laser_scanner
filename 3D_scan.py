#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
from picamera import PiCamera
#sudo apt-get install python-camera python3-picamera

#stepper setup
microsteps = 8
step = 1.8/microsteps #deg
steppause = 0.005
rotation_dist = 9 # has to be int !!!
rotation_steps = int(rotation_dist/step) # has to be int !!!

#setup measurements
camera_laser_dist = 12.25
camera_plattform_dist = 23
DEG2RAD = np.pi/180

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
res = (2592, 1944)
camera = PiCamera(resolution=res, framerate=2)
camera.iso = 200
sleep(2)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
camera.awb_gains = camera.awb_gains
camera.awb_mode = 'off'

#video
#camera.start_recording('/home/pi/camera/video.h264')
#sleep(5)
#camera.stop_recording()

#image
#camera.capture('/home/pi/picture%02d.jpg' % 1)
#img = np.empty((res[0], res[1], 3), dtype=np.uint8)

#image and distorsions
img = np.empty((res[0], res[1], 3), dtype=np.uint8)
from cv2 import remap
mapx = np.load('/home/pi/3Dscanner/mapx.npy')
mapy = np.load('/home/pi/3Dscanner/mapy.npy')

### results
datapoints = []

### process images
'''
gauss = lambda x,a,x0,sigma: a*np.exp(-(x-x0)**2/(2*sigma**2))
from scipy.optimize import curve_fit
def find_maxima_gauss(redIMG):
    dim = np.shape(redIMG)
    maxima = np.full(dim[0],np.nan, dtype=np.float32)
    x = np.arange(dim[1])
    for k in range(dim[0]):
        try:
            if np.max(redIMG[k,:])<1:
                continue
            popt,pcov = curve_fit(gauss,x,redIMG[k,:],p0=[250,dim[1]/2,20])
            maxima[k] = popt[1]
        except:
            pass
    return maxima
'''

def find_maxima_peak(redIMG):
    dim = np.shape(redIMG)
    maxima = np.full(dim[0],np.nan, dtype=np.float32)
    for iii in range(0,dim[0]):
        index = np.argmax(redIMG[iii,:])
        if index != 0:
            maxima[iii] = index
    return maxima

from scipy.ndimage import gaussian_filter
def processing(redIMG):
    #gaussian blurr
    redIMG = gaussian_filter(redIMG,sigma=0.5)
    #threshold image
    threshold = 0.5*np.max(redIMG)
    thresholded = np.multiply(redIMG,redIMG>threshold)
    #find maxima with gaussian function (fit)
#    maxima = find_maxima_gauss(thresholded)
    maxima = find_maxima_peak(thresholded)
    #remove outliners
    dim = np.shape(redIMG)
    window = dim[1]//10 #max horizontal step
    w=20 #window size
    for p in range(len(maxima)-w):
        av = np.nanmean(maxima[p:p+w]) #average position inside window
        if (maxima[p+w//2] - av) > window:
            maxima[p+w//2] = np.nan
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
GPIO.output(5,1)
GPIO.output(6,1)

sleep(1)

#confirm successful calibration, take calibration image
GPIO.wait_for_edge(13, GPIO.FALLING)
#capture image
##camera.capture('/home/pi/3Dscanner/calibration.jpg')
#pointcloud generation
IMG = camera.capture(img, "rgb")
redIMG = IMG[:,:,0] - (IMG[:,:,1]+IMG[:,:,2])/2
redIMG = cv2.remap(redIMG,mapx,mapy,cv2.INTER_CUBIC)
maxima = processing(redIMG)
zero_shift = np.nanmean( maxima )
#x = maxima - zero_shift
#y = np.where(~np.isnan(maxima))
#z = np.round( x * camera_plattform_dist/camera_laser_dist ,2)
#for ind in np.nditer(y):
#    datapoints.append(str(x[ind])+' '+str(ind)+' '+str(z[ind]))

sleep(1)

#remove calibration screen, start scanning
GPIO.wait_for_edge(13, GPIO.FALLING)
GPIO.output(5,0)
GPIO.output(6,0)

#rotate and take pictures
GPIO.output(24,0) #enable motor
for deg in range(0,360,rotation_dist):
    
    #rotate stepper
    rotate()
    
    #first image (left laser)
    GPIO.output(5,1)
    #capture image
    ##camera.capture('/home/pi/3Dscanner/img_l_%03d.jpg' % deg)
    #pointcloud generation
    IMG = camera.capture(img, "rgb")
    redIMG = IMG[:,:,0] - (IMG[:,:,1]+IMG[:,:,2])/2
    redIMG = cv2.remap(redIMG,mapx,mapy,cv2.INTER_CUBIC)
    maxima = processing(redIMG)
    #zero_shift = np.nanmean( maxima )
    x = maxima - zero_shift
    y = np.where(~np.isnan(maxima))
    z = np.round( x * camera_plattform_dist/camera_laser_dist ,2)
    for ind in np.nditer(y):
        xx = x[ind]*np.cos(deg*DEG2RAD)
        yy = ind
        zz = z[ind]*np.sin(deg*DEG2RAD)
        datapoints.append(str(xx)+' '+str(yy)+' '+str(zz))
    GPIO.output(5,0)
    
    #second image (right laser)
    GPIO.output(6,1)
    #capture image
    ##camera.capture('/home/pi/3Dscanner/img_r_%03d.jpg' % deg)
    #pointcloud generation
    IMG = camera.capture(img, "rgb")
    redIMG = IMG[:,:,0] - (IMG[:,:,1]+IMG[:,:,2])/2
    redIMG = cv2.remap(redIMG,mapx,mapy,cv2.INTER_CUBIC)
    maxima = processing(redIMG)
    #zero_shift = np.nanmean( maxima )
    x = maxima - zero_shift
    y = np.where(~np.isnan(maxima))
    z = - np.round( x * camera_plattform_dist/camera_laser_dist ,2)
    for ind in np.nditer(y):
        xx = x[ind]*np.cos(deg*DEG2RAD)
        yy = ind
        zz = z[ind]*np.sin(deg*DEG2RAD)
        datapoints.append(str(xx)+' '+str(yy)+' '+str(zz))
    GPIO.output(6,0)
GPIO.output(24,1) #disable motor
save_point_cloud(datapoints)

#restart the script itself
import subprocess
subprocess.call(['sudo /etc/init.d/3D_scan.sh start'], shell=False)