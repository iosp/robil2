## Installation

### Dependencies
```
sudo apt-get install ros-$ROS_DISTRO-mavros protobuf-compiler
(install pip: $ sudo apt-get install python-pip python-dev build-essential )
sudo pip install numpy toml
```

### Prepare source code
```
cd [PATH_TO_ROBIL_WS]/src
git clone https://github.com/PX4/Firmware.git -b v1.7.3
```

### Compile catkin workspace
```
sudo apt install ros-kinetic-diagnostic-aggregator 
sudo apt install ros-kinetic-pcl-ros
sudo apt-get install python-jinja2 
cd [PATH_TO_ROBIL_WS]
catkin_make
```

### Compile px4 gazebo plugin
```
cd [PATH_TO_ROBIL_WS]/src/Firmware
make posix_sitl_default gazebo
```
This last command compiles and run gazebo including a drone.

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

Note: 
You have to be super user in order to run this application:
```
sudo usermod -a -G dialout robil
```
You have also to remove the modemmanager since it interferes:
```
sudo apt-get remove modemmanager
```

### Takeoff
To perform a take off, first arm the vehicle by clicking 
on "Disarmed" button on the top toolbar, then click on 
"Take off" button on the left sidebar.

### Goto point
Left click on the map

## ROS
All drone's data published via ROS topics, check ```/agent1/mavros/*``` topics

## Gazebo

### Spawn drone

First start gazebo simulation, then you can spawn a drone:
```
roslaunch robil_drone_model spawn_robil_drone.launch
```
