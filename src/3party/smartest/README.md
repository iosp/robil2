# smartest package (SmARTest)
This package contains the code of SmARTest (Simulated Autonomous Robotic Testing) 
a tool for automatic testing and verification of safety of autonomous vehicles.


This package had been adjusted to test the performances of Robil2 UGV: 
Ubuntu 14.04 ROS Jade Gazebo 7.

To compile the package: 
 -install ros-jade-fcl in addition to everything that was installed for Robil2.
 -compile
 
To run the pkg, first you need to specify the Robil2 and SmARTest models paths 
by editing the files in the resources directory: `smartest/resource`
you should only update the "<robot_components_models>" and "<world_components_models>" tags


##### To generate a single scenario :

`$ roslaunch smartest genScenario_robil2_tracked.launch`


the scenario is stored in the ~/Robil2/src/3party/smartest/work_space folder in a folder named scenario_1





##### To run a scenario :


`$ roslaunch smartest runScenario_robil2_tracked.launch scen:=scenario_1` 


scen can be used to choose any other scenario in the work_space folder




##### To run a replay of a scenario: 
`$ roslaunch smartest repScenario_robil2_tracked.launch scen:=scenario_1` 

scen can be used to choose any other scenario in the work_space folder



##### To run multiple scenarios :
`$ roslaunch smartest runMultipleScenario_robil2_tracked.launch n:=10`

n specifies the numer of scenarios that will be generated and performed

the scenarios will be stored in the work_space folder with the names: sampl_1 ... sample_n,  

After complition of scenario generation and execution, a summary file will be created in the work_space folder.

