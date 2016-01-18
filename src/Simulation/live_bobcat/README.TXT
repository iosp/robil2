SETUP
-----
run:
catkin_make install
rosrun live_bobcat setupEnv
RUN
-----
roslaunch the files in the launch folder

Starting version tagged Bobtank:
===================================
* All the models are located in a directory named model_sdf in the package live_bobcat.
* No need for catkin_make install\\ 
* No need for rosrun live_bobcat setupEnv
* delete the following directories from ~/.gazebo/models: GPS_INS/ FLEA3* IBEO/ wire_sensor* sick/ bobcat bobtank meshes
* In order to run the bobcat, you can use the following calls:

	roslaunch live_bobcat bobcat_empty_world.launch
	roslaunch live_bobcat TestWorld[1-6].launch

* In order to run the bobtank, you can use the following launch files:

	roslaunch live_bobcat bobtank_empty_world.launch
	roslaunch live_bobcat bobtank_TestWorld4.launch


