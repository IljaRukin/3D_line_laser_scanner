# 3D_line_laser_scanner
3D scanner build from a raspberry pi zero with camera, 2x line laser pointers and a rotating plattform.

Calibration must be improved, since the 3d scan is quite inaccurate.

# steps to use this 3D scanner

1. print the 3D files and cut some rods to size

2. assemble it with a nema17 motor, a raspberry pi (zero w) and two line lasers
```
GPIO5 #left laser
GPIO6 #right laser
GPIO13 #button
GPIO22, #stepper dir
GPIO23, #stepper step
GPIO24, #stepper enable
```

3. take calibration images while holding a printed-out image of "/rectification/calibration_reference.png"
OPTIONAL: calibrate the camera using "/rectification/1_find_corners.py" and "/rectification/2_calibration.py"

4. Then you have two options:
- execute "full_scan.py" to take images and compute point-cloud on the raspberry pi at once (slooow). For this "/rectification/mapx.npy", "/rectification/mapy.npy" and "3D_scan.py" have to be transferred to /home/localuser/3Dscanner/ and make the python-script executable by "sudo chmod +x /home/localuser/3Dscanner/full_scan.py".
- execute "take_images.py" to only take images on your pi, then transfer the images to a faster pc and execute "process_images.py" to compute the point-cloud. For this "take_images.py" have to be transferred to /home/localuser/3Dscanner/ and make the python-script executable by "sudo chmod +x /home/localuser/3Dscanner/take_images.py".

5. run scanning by pressing the button (3x times in total) and following these steps after each consequent press:
-calibration: place a screen on the plattform and make shure both line lasers are vertical and meet at the center of the plattform
-setting offset: takes (automatic) a picture of the two lines to set the offsets, as both lasers are off remove the screen
-start scanning: executes the scanning