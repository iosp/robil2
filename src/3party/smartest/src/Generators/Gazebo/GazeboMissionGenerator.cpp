/*
 * GazeboMissionGenerator.cpp
 *
 *  Created on: Apr 10, 2014
 *      Author: userws1
 */

#include "Generators/Gazebo/GazeboMissionGenerator.h"
#include <iostream>
#include <fstream>
#include <Resource/ResourceHandler.h>
#include <boost/foreach.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"

#include "std_msgs/Time.h"
#include "ros/ros.h"

#include <iostream>
#include <sstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>

//#include "robil_msgs/Path.h"
#include "robil_msgs/AssignNavTask.h"
#include "nav_msgs/Odometry.h"
#include "robil_msgs/AssignMission.h"
#include "robil_msgs/AssignMissionTask.h"
#include "robil_msgs/AssignManipulatorTask.h"

#include "SFV/SFV.h"
#include "SFV/SFVpath.h"
#include "SFV/SFVplatformPose.h"

GazeboMissionGenerator::GazeboMissionGenerator() {

}

GazeboMissionGenerator::~GazeboMissionGenerator() {
	// TODO Auto-generated destructor stub
}

void GazeboMissionGenerator::SaharGenerateMissionToOCU(SFV *sfv, std::string fileName)
{
	SFVpath *sfv_Path = (SFVpath*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::Path));

	TiXmlDocument doc(fileName);
	TiXmlDeclaration * dec_XML = new TiXmlDeclaration( "1.0", "utf-8", "" );
	doc.LinkEndChild(dec_XML);
	TiXmlElement * DataSourceXML = new TiXmlElement( "DataSource" );
	DataSourceXML->SetAttribute("xmlns:xsi","http://www.w3.org/2001/XMLSchema-instance");
	//DataSourceXML->SetAttribute("temp","temp1");
	doc.LinkEndChild(DataSourceXML);

	TiXmlElement *Missions_XML = new TiXmlElement ("Missions");
	DataSourceXML->LinkEndChild(Missions_XML);
	TiXmlElement *Mission_XML = new TiXmlElement ("Mission");
	Missions_XML->LinkEndChild(Mission_XML);

		TiXmlElement *MissionID_XML = new  TiXmlElement ("MissionID");
		TiXmlText * MissionID_value = new TiXmlText("0");
		MissionID_XML->LinkEndChild(MissionID_value);
		Mission_XML->LinkEndChild(MissionID_XML);

		TiXmlElement *MissionAssignerID_XML = new  TiXmlElement ("MissionAssignerID");
		TiXmlText * MissionAssignerID_value = new TiXmlText("0");
		MissionAssignerID_XML->LinkEndChild(MissionAssignerID_value);
		Mission_XML->LinkEndChild(MissionAssignerID_XML);

		TiXmlElement *MissionExecuterID_XML = new  TiXmlElement ("MissionExecuterID");
		TiXmlText * MissionExecuterID_value = new TiXmlText("0");
		MissionExecuterID_XML->LinkEndChild(MissionExecuterID_value);
		Mission_XML->LinkEndChild(MissionExecuterID_XML);

		TiXmlElement *PresenceVector_XML = new  TiXmlElement ("PresenceVector");
		TiXmlText * PresenceVector_value = new TiXmlText("0");
		PresenceVector_XML->LinkEndChild(PresenceVector_value);
		Mission_XML->LinkEndChild(PresenceVector_XML);

		TiXmlElement *AutonomyLevel_XML = new  TiXmlElement ("AutonomyLevel");
		TiXmlText * AutonomyLevel_value = new TiXmlText("Manual");
		AutonomyLevel_XML->LinkEndChild(AutonomyLevel_value);
		Mission_XML->LinkEndChild(AutonomyLevel_XML);

		TiXmlElement *UnmannedSystem_XML = new  TiXmlElement ("UnmannedSystem");
		TiXmlText * UnmannedSystem_value = new TiXmlText("0");
		UnmannedSystem_XML->LinkEndChild(UnmannedSystem_value);
		Mission_XML->LinkEndChild(UnmannedSystem_XML);

		TiXmlElement *WeatherConditionm_XML = new  TiXmlElement ("WeatherCondition");
		TiXmlText * WeatherCondition_value = new TiXmlText("Sunny");
		WeatherConditionm_XML->LinkEndChild(WeatherCondition_value);
		Mission_XML->LinkEndChild(WeatherConditionm_XML);

		TiXmlElement *Missions_Duration_XML = new  TiXmlElement ("Duration");
		TiXmlText * Missions_Duration_value = new TiXmlText("0");
		Missions_Duration_XML->LinkEndChild(Missions_Duration_value);
		Mission_XML->LinkEndChild(Missions_Duration_XML);

		TiXmlElement *OperationalArea_XML = new  TiXmlElement ("OperationalArea");
		TiXmlText * OperationalArea_value = new TiXmlText("0");
		OperationalArea_XML->LinkEndChild(OperationalArea_value);
		Mission_XML->LinkEndChild(OperationalArea_XML);
		TiXmlElement *ParentMissionID_XML = new  TiXmlElement ("ParentMissionID");
		TiXmlText * ParentMissionID_value = new TiXmlText("0");
		ParentMissionID_XML->LinkEndChild(ParentMissionID_value);
		Mission_XML->LinkEndChild(ParentMissionID_XML);

		TiXmlElement *MissionStartID_XML = new  TiXmlElement ("MissionStartID");
		TiXmlText * MissionStartID_value = new TiXmlText("10");
		MissionStartID_XML->LinkEndChild(MissionStartID_value);
		Mission_XML->LinkEndChild(MissionStartID_XML);


		TiXmlElement *MissionEndID_XML = new  TiXmlElement ("MissionEndID");
		Mission_XML->LinkEndChild(MissionEndID_XML);

			TiXmlElement *unsignedShort_XML = new  TiXmlElement ("unsignedShort");
			TiXmlText * unsignedShort_value = new TiXmlText("20");
			unsignedShort_XML->LinkEndChild(unsignedShort_value);
			MissionEndID_XML->LinkEndChild(unsignedShort_XML);

		TiXmlElement *Tasks_XML = new TiXmlElement("Tasks");
		Mission_XML->LinkEndChild(Tasks_XML);

			TiXmlElement *Start_Task_XML = new TiXmlElement("Task");
			Start_Task_XML->SetAttribute("xsi:type","TaskStart");
			Tasks_XML->LinkEndChild(Start_Task_XML);

				TiXmlElement *TaskType_XML = new  TiXmlElement ("TaskType");
				TiXmlText * TaskType_value = new TiXmlText("start");
				TaskType_XML->LinkEndChild(TaskType_value);
				Start_Task_XML->LinkEndChild(TaskType_XML);

				TiXmlElement *TaskID_XML = new  TiXmlElement ("TaskID");
				TiXmlText * TaskID_value = new TiXmlText("10");
				TaskID_XML->LinkEndChild(TaskID_value);
				Start_Task_XML->LinkEndChild(TaskID_XML);

				TiXmlElement *TaskAssignerID_XML = new  TiXmlElement ("TaskAssignerID");
				TiXmlText * TaskAssignerID_value = new TiXmlText("0");
				TaskAssignerID_XML->LinkEndChild(TaskAssignerID_value);
				Start_Task_XML->LinkEndChild(TaskAssignerID_XML);

				TiXmlElement *TaskExecuterID_XML = new  TiXmlElement ("TaskExecuterID");
				TiXmlText * TaskExecuterID_value = new TiXmlText("0");
				TaskExecuterID_XML->LinkEndChild(TaskExecuterID_value);
				Start_Task_XML->LinkEndChild(TaskExecuterID_XML);

				TiXmlElement *Task_Duration_XML = new  TiXmlElement ("Duration");
				TiXmlText * Task_Duration_value = new TiXmlText("0");
				Task_Duration_XML->LinkEndChild(Task_Duration_value);
				Start_Task_XML->LinkEndChild(Task_Duration_XML);
/*
	TiXmlElement *StepIDs_XML = new  TiXmlElement ("StepIDs");
	Start_Task_XML->LinkEndChild(StepIDs_XML);
*/


/*

	TiXmlElement *Task_XML = new  TiXmlElement ("Task");
	Tasks_XML->SetAttribute("xsi:type","TaskNav");
	Tasks_XML->LinkEndChild(Task_XML);


	TiXmlElement *TaskType_XML = new  TiXmlElement ("TaskType");
	TiXmlText * TaskType_value = new TiXmlText("Nav");
	TaskType_XML->LinkEndChild(TaskType_value);
	Task_XML->LinkEndChild(TaskType_XML);

	TiXmlElement *TaskID_XML = new  TiXmlElement ("TaskID");
	TiXmlText * TaskID_value = new TiXmlText("Nav");
	TaskID_XML->LinkEndChild(TaskID_value);
	Tasks_XML->LinkEndChild(TaskID_XML);

	TiXmlElement *TaskID_XML = new  TiXmlElement ("TaskID");
	TiXmlText * TaskID_value = new TiXmlText("10005");
	TaskID_XML->LinkEndChild(TaskID_value);
	Tasks_XML->LinkEndChild(TaskID_XML);

	TiXmlElement *TaskAssignerID_XML = new  TiXmlElement ("TaskAssignerID");
	TiXmlText * TaskAssignerID_value = new TiXmlText("10005");
	TaskAssignerID_XML->LinkEndChild(TaskAssignerID_value);
	Tasks_XML->LinkEndChild(TaskAssignerID_XML);

	TiXmlElement *TaskExecuterID_XML = new  TiXmlElement ("TaskExecuterID");
	TiXmlText * TaskExecuterID_value = new TiXmlText("0");
	TaskExecuterID_XML->LinkEndChild(TaskExecuterID_value);
	Tasks_XML->LinkEndChild(TaskExecuterID_XML);

	TiXmlElement *Duration_XML = new  TiXmlElement ("Duration");
	TiXmlText * TaskExecuterID_value = new TiXmlText("0");
	Duration_XML->LinkEndChild(TaskExecuterID_value);
	Tasks_XML->LinkEndChild(Duration_XML);

	TiXmlElement *NavPath_XML = new  TiXmlElement ("NavPath");
	Tasks_XML->LinkEndChild(NavPath_XML);

*/

/*
	<NavPath>
	            <Points>
	              <Point ID="11">
	                <Latitude>32.0110696593482</Latitude>
	                <Longitude>34.9118124912161</Longitude>
	                <Altitude>0</Altitude>
	                <Heading>0</Heading>
	                <Speed>5</Speed>
	                <Action>None</Action>
	                <Feature>
	                  <Feature>
	                    <FeatureClass>ObjectID</FeatureClass>
	                    <DataType>LongInteger</DataType>
	                    <Value>11</Value>
	                  </Feature>
	                  <Feature>
	                    <FeatureClass>ObjectClassID</FeatureClass>
	                    <DataType>ShortInteger</DataType>
	                    <Value>4</Value>
	                  </Feature>
	                  <Feature>
	                    <FeatureClass>Speed</FeatureClass>
	                    <DataType>Float</DataType>
	                    <Value>0</Value>
	                  </Feature>
	                </Feature>
	              </Point>
*/

	doc.SaveFile(fileName.c_str());
}
void GazeboMissionGenerator::generateMission(SFV *sfv, std::string fileName)
{
	SFVplatformPose *sfv_PlatPose = (SFVplatformPose*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::platform_pose));
	SFVpath *sfv_Path = (SFVpath*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::Path));

	std::ofstream file;
	file.open (fileName + ".txt");
	file<< "PLATFORM" <<std::endl;
	file<< "koko" <<std::endl;
	file<< "START"<<std::endl;

	float plat_x = sfv_PlatPose->get_PlatInit_xy()->at('x');
	float plat_y = sfv_PlatPose->get_PlatInit_xy()->at('y');
	file<< plat_x <<" " <<plat_y << " " << std::endl;

	file<< "WAYPOINTS"<<std::endl;
	float x, y;
	for(SFVwp* wp_it : *(sfv_Path->get_PathWPs()))
	{
		x=wp_it->get_WPxy()->at('x');
		y=wp_it->get_WPxy()->at('y');
		file<< x <<" " <<y << " " << 3 << std::endl;
	}
	file.close();
}

void GazeboMissionGenerator::generateMission_ROBIL2(SFV * sfv,std::string fileName)
{
	SFVpath *sfv_Path = (SFVpath*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::Path));
	SFVplatformPose *sfv_PlatPose = (SFVplatformPose*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::platform_pose));

	rosbag::Bag bag;
	bag.open(fileName + ".bag", rosbag::bagmode::Write);

////

	robil_msgs::AssignManipulatorTask robilManipulatioTask;
	robilManipulatioTask.header.seq = 1;
	robilManipulatioTask.task_id = "10";
	robilManipulatioTask.task_description = "raising of the arm task";

	robil_msgs::AssignManipulatorTaskStep robilManipulationTaskStep;
	robilManipulationTaskStep.id = 1;
	robilManipulationTaskStep.type = 1;
	robilManipulationTaskStep.value = 0.2;
	robilManipulationTaskStep.blade_relativity = 0;
	robilManipulationTaskStep.success_timeout = 5;
	robilManipulationTaskStep.duration_at_end = 2;
	robilManipulatioTask.steps.push_back(robilManipulationTaskStep);

	ros::Time dumy_play_time(0,1);
	bag.write("/OCU/SMME/ManipulationTask",dumy_play_time,robilManipulatioTask);  // the SMME need one task to weak up
	ros::Time man_task_play_time(1,0);
	bag.write("/OCU/SMME/ManipulationTask",man_task_play_time,robilManipulatioTask);

////

	robil_msgs::AssignNavTask robilNavTask;
	robilNavTask.header.seq = 2;
	robilNavTask.task_id = "20";
	robilNavTask.task_description = "navigation task";

	nav_msgs::Odometry wp;
	float plat_init_x = sfv_PlatPose->get_PlatInit_xy()->at('x');
	float plat_init_y = sfv_PlatPose->get_PlatInit_xy()->at('y');
	for(SFVwp* wp_it : *(sfv_Path->get_PathWPs()))
	{
		wp.pose.pose.position.x = wp_it->get_WPxy()->at('x') - plat_init_x;
		wp.pose.pose.position.y = wp_it->get_WPxy()->at('y') - plat_init_y;
		robilNavTask.waypoints.push_back(wp);
	}


	ros::Time nav_task_play_time(2,0);
	bag.write("/OCU/SMME/NavigationTask",nav_task_play_time,robilNavTask);


/////

	robil_msgs::AssignMission robilMission;
	robilMission.mission_description = "navigation task";
	robilMission.mission_id = "1001";

	robil_msgs::AssignMissionTask mission_task;
	mission_task.task_id = "10";
	robilMission.tasks.push_back(mission_task);
	mission_task.task_id = "20";
	robilMission.tasks.push_back(mission_task);

	ros::Time mission_play_time(3,0);
	bag.write("/OCU/SMME/MissionPlan",mission_play_time,robilMission);

////

	std_msgs::String play;
	play.data = "/mission/1001/StartMission";
	ros::Time play_time(4,0);
	bag.write("/decision_making/events",play_time,play);
	ros::Time play2_time(5,0);
	bag.write("/decision_making/events",play2_time,play);   // one play some times not enoth


	bag.close();
}



void GazeboMissionGenerator::generate(SFV * sfv , std::string scenario_folder_url)
{
	std::string temp = scenario_folder_url+"/scenarioMission";

	std::cout << "\033[1;36m Producing " << temp << ".txt \033[1;36m" << std::endl;
	generateMission(sfv, temp);

	std::cout << "\033[1;36m Producing " << temp << ".bag \033[1;36m" << std::endl;
	generateMission_ROBIL2(sfv, temp);


	std::cout << "\033[1;36m Producing " << temp << ".xml \033[1;36m" << std::endl;
	SaharGenerateMissionToOCU(sfv, temp);

}

