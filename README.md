# Install:
Branch **master**: 
1. Runs on Ubuntu 16.04 with ROS Kinetic
2. Runs also on Ubuntu 14.04 eith ROS Jade
3. Checked on Ubuntu 16.04 according https://github.com/iosp/robil2/wiki/Acceptance-Tests
4. There are some issues with fcl (kinetic) and therefore, the following should be done:
       * install ros-kinetic-fcl-catkin
       * install with synaptic version fcl 0.5 and version fcl dev 0.5
5. See below Toward moving to kinetic - branch toKinetic section 5 for OCU

# Run simulation
roslaunch live_bobcat bobcat_tracked_empty_world.launch 
or 
bobcat_tracked_Robil_Testing_grounds.launch
bobcat_tracked_stair_world.launch
bobcat_tracked_TestWorld4.launch
bobcat_tracked_TestWorld.launch

# Run robot
roslaunch robil2conf frameworkInit.launch

# Run OCU
Open the file: $ROBIL2/src/Framework/ocu/src/RobilOCU.html on a browser.
Note You have to refresh the OCU each time the robot reboots.


************************************************************************************************
# OBSOLETE - Toward moving to kinetic - branch toKinetic - 
1. Perception crashes
2. NO ==>Install Flexible Collision Library from source: https://github.com/flexible-collision-library/fcl
3. Install version 0.5 + dev of FCL (with synaptic)
4. For OCU: 
     * Install: sudo apt-get install ros-kinetic-rosbridge-server\\
     * Install: sudo apt-get install ros-kinetic-tf2-web-republisher
     * Roslibjs: located in ~/robil2/src/3party 
     * For more details: http://www.robil.org/trac_robil/wiki/RobilOCU
     
# smartest
See relevant README
 -install ros-jade-fcl in addition to everything that was installed for Robil2.
# navex
 -install ros-jade-ackerman-msgs

# robil2
This repository contains the code of robil2.
The site of the project is www.robil.org
******************************************************************
Starting tag: https://github.com/iosp/robil2/releases/tag/New_Tracked_Model:
There are two fully checked models: 
   - with wheels or 
   - with tracks 
   - but with a static arm.

Relevant launch files:
   * For wheels:
      * roslaunch live_bobcat bobcat_static_arm_empty_world.launch
      * roslaunch live_bobcat bobcat_static_arm_stair_world.launch 
      * roslaunch live_bobcat bobcat_static_arm_TestWorld.launch
      * roslaunch live_bobcat bobcat_static_arm_TestWorld4.launch
   * For tracked model:
      * roslaunch live_bobcat bobcat_tracked_static_arm_empty_world.launch
      * roslaunch live_bobcat bobcat_tracked_static_arm_stair_world.launch 
      * roslaunch live_bobcat bobcat_tracked_static_arm_TestWorld.launch
      * roslaunch live_bobcat bobcat_tracked_static_arm_TestWorld4.launch

******************************************************************
Starting tag: https://github.com/iosp/robil2/releases/tag/Wheels_Stable_Model:
- The launch files that are relevant:
- 
*roslaunch live_bobcat bobcat_static_arm_TestWorld4.launch 

*roslaunch live_bobcat bobcat_static_arm_empty_world.launch

*roslaunch live_bobcat bobcat_static_arm_stair_world.launch
- This version supports migration to Gazebo 5 and ROS Jade: see http://www.robil.org/trac_robil/wiki/MovingToJade
