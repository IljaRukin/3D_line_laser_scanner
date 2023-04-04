import RPi.GPIO as GPIO
from time import sleep
from picamera import PiCamera

microsteps = 8
steppause = 0.001

#GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)

GPIO.setup(22, GPIO.OUT) #dir
GPIO.setup(23, GPIO.OUT) #step
GPIO.setup(24, GPIO.OUT) #enable
GPIO.output(22,0) #clockwise
GPIO.output(23,0) #pull down step
GPIO.output(24,1) #disable

#turn laser off
GPIO.output(5,0)
GPIO.output(6,0)

#trigger-button
#GPIO.wait_for_edge(13, GPIO.FALLING)

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
#img = np.empty((res[0], res[1], 3), dtype=np.uint8)

#stepper rotation
def rotate():
    steps = 5*microsteps #deg=5*1.8=9
    GPIO.output(24,0) #enable
    sleep(0.1)
    for i in range(steps):
        GPIO.output(23,1)
        sleep(steppause)
        GPIO.output(23,0)
        sleep(steppause)
    sleep(0.5)
    GPIO.output(24,1) #disable
    return None

#zero-th image (calibration)
GPIO.output(5,1)
GPIO.output(6,1)
camera.capture('/home/pi/camera/calibration.jpg')
GPIO.output(5,0)
GPIO.output(6,0)

#rotate and make picture
rotation = 9 #deg
for deg in range(0,360,rotation):
    
    print(deg)
    
    #rotate stepper
    rotate()
    
    #first image
    GPIO.output(5,1)
    GPIO.output(6,0)
    #img = camera.capture(img, "bgr")
    camera.capture('/home/pi/camera/img_l_%02d.jpg' % deg)
    
    #second image
    GPIO.output(5,0)
    GPIO.output(6,1)
    #img = camera.capture(img, "bgr")
    camera.capture('/home/pi/camera/img_r_%02d.jpg' % deg)
    
    GPIO.output(6,0)
