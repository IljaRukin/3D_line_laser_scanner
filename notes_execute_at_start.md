0.make "take_images.py" executable
```
sudo chmod +x /home/localuser/3Dscanner/take_images.py
```
1. make "take_images.py" execute at restart
```
sudo nano /etc/init.d/take_images.sh
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
    echo "Starting take_images.py"
    //home/localuser/3Dscanner/take_images.py &
    ;;
  stop)
    echo "Stopping take_images.py"
    pkill -f /home/localuser/3Dscanner/take_images.py
    ;;
  *)
    echo "Usage: /home/localuser/3Dscanner/take_images.py {start|stop}"
    exit 1
    ;;
esac

exit 0
```
```
sudo chmod +x /etc/init.d/take_images.sh
sudo update-rc.d take_images.sh defaults
```
start script (for first time only):
```
sudo /etc/init.d/take_images.sh start
```