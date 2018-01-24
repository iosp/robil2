/*
 * GazeboExecutor.cpp
 *
 *  Created on: Jul 7, 2014
 *      Author: userws3
 */

#include <string>
#include <iostream>

#include "Executor/GazeboExecutor.h"
#include "Executor/scenarioLauncher.h"
#include "SFV/SFV.h"
#include "Resource/ResourceHandler.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <tinyxml.h>
#include "utils/TinyXmlDef.h"

GazeboExecutor::GazeboExecutor(SFV *sfv)
{
	my_Scenario_folder_url = sfv->get_WSfolder(); //Scenario_folder_url;

	my_pyInterface = ResourceHandler::getInstance(sfv->get_ResourceFile()).getRobotPyInterface();
    std::cout << "my_pyInterface = " << my_pyInterface << std::endl;

	was_executed_flag = false;

	my_launcher = new ScenarioLauncher(my_pyInterface);
	my_launcher->start_launcher();
}

GazeboExecutor::~GazeboExecutor()
{

	my_launcher->stop_launcher();
	my_launcher->~ScenarioLauncher();
}

//double temp_grade;
std_msgs::Float32MultiArray::ConstPtr temp_grades;
void scen_grade_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	temp_grades = msg ;
}

int GazeboExecutor::RunScenario(int argc, char** argv)
{
    std::cout << " my_Scenario_folder_url = "<< my_Scenario_folder_url << std::endl;

    ros::init(argc, argv, "test_node");
	ros::NodeHandle n;
	ros::Subscriber model_states_sub;

	bool end_scen_flag = false;

	my_launcher->setScenarioEnv(my_Scenario_folder_url);
	std::cout << "\033[1;35m Scenario Env_Variables are loaded !!! \033[0m" << std::endl;

	my_launcher->startGazeboServer(my_Scenario_folder_url);
	std::cout << "\033[1;35m GazeboServer is loaded !!! \033[0m" << std::endl;

	my_launcher->launchGazeboClient();
	std::cout << "\033[1;35m GazeboClient is loaded !!! \033[0m" << std::endl;

    my_launcher->launchPlatformControlsSpawner(); // used by the srvss_dummy
    std::cout << "\033[1;35m Platform is loaded !!! \033[0m" << std::endl;

    my_launcher->launchWPdriver(my_Scenario_folder_url);
    std::cout << "\033[1;35m WP driver is loaded !!! \033[0m" << std::endl;

    my_launcher->launchTFbroadcaster();
    std::cout << "\033[1;35m TF publishing is loaded !!! \033[0m" << std::endl;

    my_launcher->launchRecorder(my_Scenario_folder_url);
    std::cout << "\033[1;35m Recorder is loaded !!! \033[0m" << std::endl;

    my_launcher->launchGrader(my_Scenario_folder_url);
    std::cout << "\033[1;35m Grader is loaded !!! \033[0m" << std::endl;

    my_launcher->GazeboUnPause();
    std::cout << "\033[1;35m Gazebo is Playin !!! \033[0m" << std::endl;


    model_states_sub = n.subscribe("/srvss/grades", 100, scen_grade_Callback);
    ros::Duration scen_max_duration(180, 0);
    ros::Time begin_time = ros::Time::now();
    ros::Time now_time = ros::Time::now();
    while ( (! end_scen_flag) && ros::ok() && (now_time - begin_time < scen_max_duration) )
         {
        now_time = ros::Time::now();
        ros::spinOnce();
         }

    //char c;
    //std::cin >>c;

    my_scenario_graedes = temp_grades;
    was_executed_flag = true;

    my_launcher->GazeboPause();
    std::cout << "\033[1;35m Gazebo is Paused !!! \033[0m" << std::endl;

    my_launcher->launchPlatformControlsUnspawner(); // used by the srvss_dummy
    std::cout << "\033[1;35m Platform controls are unspawnnd !!! \033[0m" << std::endl;

    my_launcher->stop_launcher();

	return 1;
}


