In order to run:

rosrun viso_isl_ros stereo_odometer or
rosrun viso_isl_ros stereo_odometer stereo:=narrow_stereo image:=image_rect

Works with gazebo:

connects to:
	/SENSORS/CAM/L
	/SENSORS/CAM/L/camera_info
	/SENSORS/CAM/R
	/SENSORS/CAM/R/camera_info

This package needs rectified images, meaning that the "y" of both images is the same (the both cameras are at the same level)

Debug:
catkin_make -DCMAKE_BUILD_TYPE=Debug
rosrun --prefix "xterm -e gdb --args"  viso_isl_ros stereo_odometer

Testing ROS:
In order to test the infrastructure, set #define DEBUG in files:
	./viso_isl_ros/src/odometer_base.h
	./viso_isl_ros/src/stereo_odometer.cpp


