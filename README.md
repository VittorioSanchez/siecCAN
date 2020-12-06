# siecCAN

Scripts that implements communication between the CAN bus and ROS on the Raspberry Pi.

The script **listener.py** is the one that manages all this. 

The other scripts are used for testing, when *listener.py* is running:

- **data_log.py**: allows to save data in files in order to plot it on Excel afterwards
- **publisher_direction.py**: script used to send angular speed commands
- **publisher_speed.py**: script used to send speed commands

## How to launch the project properly

1. Launch the car

2. Initialise CAN bus
```
sudo /sbin/ip link set can0 up type can bitrate 400000
```
3. Launch ROS master
```
roscore
```
4. Launch *rosbridge_server*
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
5. Launch *listener.py*

6. If the HMI is installed (*see hmi repo*) you can access it typing *localhost* in your browser, else you can use the scripts named below to send commands

*Note: that's the painful way, see **siec_roslaunch** repo to use an easier method.*
