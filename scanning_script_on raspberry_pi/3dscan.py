import RPi.GPIO as GPIO
from time import sleep
from picamera import PiCamera

#GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)

#turn laser off
GPIO.output(5,0)
GPIO.output(6,0)

#trigger-button
GPIO.wait_for_edge(13, GPIO.FALLING)

#camera setup
res = (2592, 1944)
camera = PiCamera(resolution=res, framerate=2)
camera.iso = 100
sleep(2)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g

#image and distorsions
img = np.empty((res[0], res[1], 3), dtype=np.uint8)
from cv2 import remap
mapx = np.load('raspi/mapx.npy')
mapy = np.load('raspi/mapy.npy')

#3d data points
import time
import datetime
ts = time.time()
current_time = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
f = open("3dpoints"+str(current_time)+".txt", "w")
f.write("#point cloud scan")

#image processing
import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.optimize import curve_fit
normalize = lambda x: (x - np.min(x))/(np.max(x)-np.min(x))*255
gauss = lambda x,a,x0,sigma: a*np.exp(-(x-x0)**2/(2*sigma**2))
def interpolate_position(image):
    dim = np.shape(image)
    maximas = np.full(dim[0],np.nan, dtype=np.float32)
    x = np.arange(dim[1])
    for k in range(dim[0]):
        try:
            if np.max(image[k,:])<1:
                continue
            popt,pcov = curve_fit(gauss,x,image[k,:],p0=[250,dim[1]/2,20])
            maximas[k] = popt[1]
        except:
            pass
    return maximas
def processing(img):
    #extract red
    red_pic = img[:,:,2]
    #gaussian blurr
    red_pic = gaussian_filter(red_pic,sigma=0.5)
    #threshold image
    threshold = 0.8*np.max(red_pic)
    thresholded = np.multiply(red_pic,red_pic>threshold)
    #find maximas with gaussian function (fit)
    maximas = interpolate_position(thresholded)
    dim = np.shape(red_pic)
    #remove outliners
    window = dim[1]//10 #max horizontal step
    w=20 #window size
    for p in range(len(maximas)-w):
        av = np.nanmean(maximas[p:p+w]) #average position inside window
        if (maximas[p+w//2] - av) > window:
            maximas[p+w//2] = np.nan
    return maximas
def save_3dpoints(maximas):
    for point in maximas:
        coord = 'unknown'
        f.write(str(coord))
    return None

#stepper rotation
def rotate(deg):
    steps = deg *300
    return None

rotation = 10 #deg
for deg in range(0,360,rotation):
    
    #rotate stepper
    rotate(rotation)
    
    #first image
    GPIO.output(5,1)
    GPIO.output(6,0)
    img = camera.capture(img, "bgr")
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_CUBIC)
    maximas = processing(dst)
    save_3dpoints(maximas)
    
    #second image
    GPIO.output(5,0)
    GPIO.output(6,1)
    img = camera.capture(img, "bgr")
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_CUBIC)
    maximas = processing(dst)
    save_3dpoints(maximas)

f.close()
