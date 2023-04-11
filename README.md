# 3D_line_laser_scanner
3D scanner build from a raspberry pi zero with camera, 2x line laser pointers and a rotating plattform.

# steps to use this 3D scanner
1. print the 3D files and cut some rods to size
2. assemble it with a nema17 motor, a raspberry pi (zero w) and two line lasers
```
GPIO5 #left laser
GPIO6 #right laser
GPIO 13 #button
GPIO22, #stepper dir
GPIO23, #stepper step
GPIO24, #stepper enable
```
3. take calibration images while holding a printed-out image of "/rectification/calibration_reference.png"
4. calibrate the camera using "/rectification/1_find_corners.py" and "/rectification/2_calibration.py"
5. save "/rectification/mapx.npy", "/rectification/mapy.npy" and "3D_scan.py" to /home/pi/3Dscanner/ on your raspberry pi and make the script executeable:
```
sudo chmod +x /home/pi/3Dscanner/3D_scan.py
```
6. make "3D_scan.py" execute at restart
```
sudo nano /etc/init.d/3D_scan.sh
```
with contents:
```
#! /bin/sh

### BEGIN INIT INFO
# Provides:          listen-for-shutdown.py
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
### END INIT INFO

# If you want a command to always run, put it here

# Carry out specific functions when asked to by the system
case "$1" in
  start)
    echo "Starting 3D_scan.py"
    //home/pi/3Dscanner/3D_scan.py &
    ;;
  stop)
    echo "Stopping 3D_scan.py"
    pkill -f /home/pi/3Dscanner/3D_scan.py
    ;;
  *)
    echo "Usage: /home/pi/3Dscanner/3D_scan.py {start|stop}"
    exit 1
    ;;
esac

exit 0
```
```
sudo chmod +x /etc/init.d/3D_scan.sh
```
7. run scanning by pressing the button (3x times in total) and following these steps after each consequent press:
-calibration: place a screen on the plattform and make shure both line lasers are vertical and meet at the center of the plattform
-setting offset: takes (automatic) a picture of the two lines to set the offsets, as both lasers are off remove the screen
-start scanning: executes the scanning
