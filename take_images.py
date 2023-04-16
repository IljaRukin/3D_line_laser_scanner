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
#from numpy import uint8 as np_uint8
#from numpy import empty as np_empty

### create folder for images
#from os import mkdir
#mkdir('./scan_images')

#stepper setup
microsteps = 8
step = 1.8/microsteps #deg
steppause = 0.005
rotation_dist = 9 # has to be int !!!
rotation_steps = int(rotation_dist/step) # has to be int !!!

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

#video
#camera.start_recording('/home/pi/camera/video.h264')
#sleep(5)
#camera.stop_recording()

#image
#camera.capture('/home/pi/picture%02d.jpg' % 1)
#IMG = np_empty((res[0], res[1], 3), dtype=np_uint8)

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
camera.capture('/home/localuser/3Dscanner/scan_images/calibration.jpg')
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
    camera.capture('/home/localuser/3Dscanner/scan_images/img_l_%03d.jpg' % deg)
    GPIO.output(5,0)
    
    #second image (right laser)
    GPIO.output(6,1)
    #capture image
    camera.capture('/home/localuser/3Dscanner/scan_images/img_r_%03d.jpg' % deg)
    GPIO.output(6,0)
GPIO.output(24,1) #disable motor

#restart the script itself
#import subprocess
#subprocess.call(['sudo /etc/init.d/take_images.sh start'], shell=False)