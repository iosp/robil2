In order to run:

rosrun viso_isl_ros stereo_odometer

workks with gazebo:

connects to:
	/SENSORS/CAM/L
	/SENSORS/CAM/L/camera_info
	/SENSORS/CAM/R
	/SENSORS/CAM/R/camera_info

This package needs rectified images, meaning that the "y" of both images is the same (the both cameras are at the same level)
