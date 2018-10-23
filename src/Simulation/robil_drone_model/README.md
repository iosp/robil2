## Installation

### Dependencies
```
sudo apt-get install ros-$ROS_DISTRO-mavros protobuf-compiler
```

### Prepare source code
```
cd [PATH_TO_ROBIL_WS]/src
git clone https://github.com/PX4/Firmware.git -b v1.7.3
```

### Compile catkin workspace
```
cd [PATH_TO_ROBIL_WS]
catkin_make
```

### Compile px4 gazebo plugin
```
cd [PATH_TO_ROBIL_WS]/src/Firmware
make posix_sitl_default gazebo
```

## QGroundControl

### Download QGroundControl:
```
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage
```

### Run
```
./QGroundControl.AppImage
```

QGroundControl will automatically connect to drone.

### Takeoff
To perform a take off, first arm the vehicle by clicking 
on "Disarmed" button on the top toolbar, then click on 
"Take off" button on the left sidebar.

### Goto point
Left click on the map

## ROS
All drone's data published via ROS topics, check ```/agent1/mavros/*``` topics