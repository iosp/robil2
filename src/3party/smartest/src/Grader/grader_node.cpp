#include <iostream>
#include <fstream>
#include <cmath>      // std::abs()
#include <string>
#include <map>
#include <vector>
#include <ros/console.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

#include "tf/transform_listener.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"

#include "fcl/distance.h"
#include "fcl/traversal/traversal_node_bvhs.h"

#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"


#include "SFV/SFV.h"
#include "SFV/SFVobjScattering.h"
#include "SFV/SFVobsOnPathScattering.h"
#include "SFV/SFVpath.h"
#include "SFV/SFVwp.h"

#include "Resource/ResourceHandler.h"


#include "gazebo/gazebo_config.h"

#if GAZEBO_MAJOR_VERSION == 2  
#include <sdformat-1.4/sdf/sdf.hh>  //worked with ros Gazebo2

#elif GAZEBO_MAJOR_VERSION == 5  
#include <sdformat-2.3/sdf/sdf.hh> //working with ros Gazebo5

#elif GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION < 9 //working with ros Gazebo7
#include <sdformat-4.3/sdf/sdf.hh>

#elif GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION == 9  //working with ros Gazebo7.9
#include <sdformat-4.4/sdf/sdf.hh>

#endif


using namespace fcl;


#define PI 3.14159265359
#define MIN_ALLOWED_OBS_DIST 2
#define MAX_ALLOWED_ROLL_ANG (15*PI/180)
#define MAX_ALLOWED_PITCH_ANG (30*PI/180)

float scenario_obj_min_dist = 100;
float scenario_roll_max_ang = 0;
float scenario_pitch_max_ang = 0;
float scenario_goalWP_min_dis = 100;
//std::string PATH = "null" ;


/**
 *   utility function for loading *.obj file to fcl Vertices and Triangles
 */
void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
    case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        }
        else
        {
          FCL_REAL x = (FCL_REAL)atof(strtok(NULL, "\t "));
          FCL_REAL y = (FCL_REAL)atof(strtok(NULL, "\t "));
          FCL_REAL z = (FCL_REAL)atof(strtok(NULL, "\t "));
          Vec3f p(x, y, z);
          points.push_back(p);
        }
      }
      break;
    case 'f':
      {
        Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
              // vertex ID
              if(i == 0)
                v1 = data[0];
              else
                v1 = data[t + i];

              tri[i] = atoi(v1) - 1;
            }
          }
          triangles.push_back(tri);
        }
      }
      break;
    }
  }
}

/**
 *   utility function for creating fcl Quanterion from Roll, Pitch and Yaw
 */
Quaternion3f Quanterion3f_from_RPY(float Roll,float Pitch,float Yaw)
{
	tf::Matrix3x3 obs_mat;
	obs_mat.setEulerYPR(Yaw,Pitch,Roll);

	tf::Quaternion q_tf;
	obs_mat.getRotation(q_tf);

	Quaternion3f q(q_tf.getW(),q_tf.getX(),q_tf.getY(),q_tf.getZ());
	return(q);
}





// Global Variabled used by Grading function
std::map<std::string ,BVHModel<RSS> *> * obs_models_map;
std::map<std::string ,BVHModel<RSS> *> * robot_models_map;
SFV *sfv;
tf::TransformListener *listener_ptr;
boost::mutex collision_mutex;
boost::mutex tf_data_mutex;

ros::ServiceClient gazebo_GetModelState_client;
std::string robot_name;

/**
 * Grading function for collision detection
 *    TODO : collision_grader should use gazebo_msgs::GetLinkState, but this service msg don't exist in gazebo2.2
 */
void collision_grader(const ros::TimerEvent&)
{
    float min_dist = 100;

    std::string part_name;
    BVHModel<RSS> * part_model;

    gazebo_msgs::GetModelState ModelState;
	for (std::map<std::string ,BVHModel<RSS> *>::iterator RobotPart_it=robot_models_map->begin(); RobotPart_it!=robot_models_map->end(); ++RobotPart_it)
	{
    	part_name = RobotPart_it->first;
		part_model = RobotPart_it->second;


		ModelState.request.relative_entity_name = "world";
		ModelState.request.model_name = robot_name;
		gazebo_GetModelState_client.call(ModelState);

		Vec3f robot_p = Vec3f(ModelState.response.pose.position.x,ModelState.response.pose.position.y,ModelState.response.pose.position.z);
		Quaternion3f robot_q = Quaternion3f(ModelState.response.pose.orientation.w,ModelState.response.pose.orientation.x,ModelState.response.pose.orientation.y,ModelState.response.pose.orientation.z);
		Transform3f robot_pose = Transform3f(robot_q,robot_p);


		Transform3f part_in_the_world;
		tf::StampedTransform part_transform;
		try {
		   listener_ptr->waitForTransform("/body", part_name, ros::Time(0), ros::Duration(1) );
		   listener_ptr->lookupTransform("/body", part_name ,ros::Time(0), part_transform);

		   Vec3f part_p = Vec3f(part_transform.getOrigin().x(),part_transform.getOrigin().y(),part_transform.getOrigin().z());
		   Quaternion3f part_q = Quaternion3f(part_transform.getRotation().w(),part_transform.getRotation().x(),part_transform.getRotation().y(),part_transform.getRotation().z());
		   Transform3f part_pose_in_the_robot = Transform3f(part_q,part_p);

		   Transform3f part_in_the_world = robot_pose * part_pose_in_the_robot;



		   	    std::vector<SFVobsOnPathScattering*> *obsOnPathScatterings_vec = new std::vector<SFVobsOnPathScattering*>;
		   	   	if (sfv->get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType::obstacles_on_path, (std::vector<sfvSubGroup*> *)obsOnPathScatterings_vec) );
		   	   	{
					for (SFVobsOnPathScattering *obsScattering_it : *obsOnPathScatterings_vec )
					{
						if ( obsScattering_it->get_NumberOfObstaclesOnPath()->get_RolledValue() > 0 )
						{
							for (SFVObstacleOnPath *obs : *(obsScattering_it->get_ObstaclesOnPath()))
							{

								std::string obs_name = ResourceHandler::getInstance(sfv->get_ResourceFile()).getObjectById(obs->get_ObstacleType()->get_RolledValue());

								float obs_x = obs->get_Obstacle_xyz()->at('x');
								float obs_y = obs->get_Obstacle_xyz()->at('y');
								float obs_z = obs->get_Obstacle_xyz()->at('z');
								Vec3f obs_p(obs_x, obs_y, obs_z);

								Vec3f centers_dist = obs_p - robot_p;
								if ( centers_dist.length() > 100 )
								   { continue;	}

								double obs_roll = obs->get_LocationRoll()->get_RolledValue();
								double obs_pitch = obs->get_LocationPitch()->get_RolledValue();
								double obs_yaw = obs->get_LocationYaw()->get_RolledValue();
								Quaternion3f obs_q = Quanterion3f_from_RPY(obs_roll,obs_pitch,obs_yaw);

								Transform3f obs_pose(obs_q,obs_p);

								DistanceResult local_result;
								if ( (robot_models_map->find(part_name)!=robot_models_map->end()) && (obs_models_map->find(obs_name.c_str())!=obs_models_map->end()) )
								{
									distance(robot_models_map->at(part_name),part_in_the_world,obs_models_map->at(obs_name.c_str()),obs_pose,1,local_result);
									if (  min_dist > local_result.min_distance )
										min_dist = local_result.min_distance;
									//std::cout << "part_name = " << part_name << ", current = " << local_result.min_distance << std::endl;
								}
							}
						}
					}
		   	   	}


				std::vector<SFVobjScattering*> *objectsOnPathScatterings_vec = new std::vector<SFVobjScattering*>;
		   	   	if (sfv->get_VecOfSubGroupsByFeatureGroupType(ScenarioFeatureGroupType::objects, (std::vector<sfvSubGroup*> *)objectsOnPathScatterings_vec) );
		   	   	{
					for (SFVobjScattering* objScattering_it : *objectsOnPathScatterings_vec )
					{
						if ( objScattering_it->get_NumberOfObjects()->get_RolledValue() > 0 )
						{
							for (SFVObject* obj : *(objScattering_it->get_Objects()))
							{
								std::string obj_name = ResourceHandler::getInstance(sfv->get_ResourceFile()).getObjectById(obj->get_ObjectType()->get_RolledValue());

								float obj_x = obj->get_Object_xyz()->at('x');
								float obj_y = obj->get_Object_xyz()->at('y');
								float obj_z = obj->get_Object_xyz()->at('z');
								Vec3f obj_p(obj_x, obj_y, obj_z);

								Vec3f centers_dist = obj_p - robot_p;
								if ( centers_dist.length() > 10 )
								   { continue;	}

								double obj_roll = obj->get_LocationRoll()->get_RolledValue();
								double obj_pitch = obj->get_LocationPitch()->get_RolledValue();
								double obj_yaw = obj->get_LocationYaw()->get_RolledValue();
								Quaternion3f obj_q = Quanterion3f_from_RPY(obj_roll,obj_pitch,obj_yaw);

								Transform3f obj_pose(obj_q,obj_p);

								DistanceResult local_result;
								if ( (robot_models_map->find(part_name)!=robot_models_map->end()) && (obs_models_map->find(obj_name.c_str())!=obs_models_map->end()) )
								{
									distance(robot_models_map->at(part_name),part_in_the_world,obs_models_map->at(obj_name.c_str()),obj_pose,1,local_result);
									if (  min_dist > local_result.min_distance )
										min_dist = local_result.min_distance;
									//std::cout << "part_name = " << part_name << ", current = " << local_result.min_distance << std::endl;
								}

							}
						}
					}
		   	   	}
			}
		catch (tf::LookupException &ex)
		      {
		   	ROS_ERROR("%s",ex.what());
		   	continue;
		   	// return;
		      }

	}

		//std::cout << " min dist to  =  " <<  min_dist << std::endl;
		//ROS_DEBUG_NAMED("MinD2Obj", "Minimum Distance to Object  = %f",min_dist);
		//ROS_INFO(" obj_path = %s " , obj_path.c_str());
		//std::cout << " Object Path  =  " <<  PATH << std::endl;

		if ( (std::abs(min_dist - scenario_obj_min_dist) > 0.5 ) && ( scenario_obj_min_dist != 100) )  // filtering distance jumps to zero with no explanation
		 return;
		
		collision_mutex.lock();
			scenario_obj_min_dist = std::min(scenario_obj_min_dist , min_dist);
		collision_mutex.unlock();
		
}


// variable used by the Grading funtion
boost::mutex rollover_mutex;

/**
 *
 * Grading function for platform stability
 */
void rollover_grader(const ros::TimerEvent&)
{
	Quaternion3f rob_body_q;
	tf::StampedTransform rob_body_transform;

	double roll, pitch, yaw;


	   gazebo_msgs::GetModelState ModelState;

	   ModelState.request.relative_entity_name = "world";
	   ModelState.request.model_name = robot_name;
	   gazebo_GetModelState_client.call(ModelState);

	   tf::Quaternion model_q(ModelState.response.pose.orientation.x,ModelState.response.pose.orientation.y,ModelState.response.pose.orientation.z,ModelState.response.pose.orientation.w);
	   tf::Matrix3x3 mat(model_q);
	   mat.getRPY(roll, pitch, yaw);

	   //std::cout << " roll = " << roll << std::endl;
	   ROS_DEBUG_NAMED("Roll", "Roll Grade  = %f",roll);
	rollover_mutex.lock();
		scenario_roll_max_ang = std::max( scenario_roll_max_ang , std::abs((float)roll) );
		scenario_pitch_max_ang = std::max( scenario_pitch_max_ang , std::abs((float)pitch) );
	rollover_mutex.unlock();
}



// variable used by the distance_to_goalWP_grader function
boost::mutex goalWP_mutex;

/**
 *
 * Grading function for reaching goalWP
 */
void distance_to_goalWP_grader(const ros::TimerEvent&)
{
		gazebo_msgs::GetModelState ModelState;

		ModelState.request.relative_entity_name = "world";
		ModelState.request.model_name = robot_name;
		gazebo_GetModelState_client.call(ModelState);

	   float rob_x = ModelState.response.pose.position.x;
	   float rob_y = ModelState.response.pose.position.y;

	   SFVpath *sfv_Path = (SFVpath*)(sfv->get_SubGroupByFeatureGroupType(ScenarioFeatureGroupType::Path));
	   SFVwp *goalWP = sfv_Path->get_PathWPs()->back();
	   float goalWP_x = goalWP->get_WPxy()->at('x');
	   float goalWP_y = goalWP->get_WPxy()->at('y');

	   float goalWP_dis = std::sqrt((rob_x-goalWP_x)*(rob_x-goalWP_x) + (rob_y-goalWP_y)*(rob_y-goalWP_y));


	   //std::cout << " goalWP_dis = " << goalWP_dis << std::endl;
		ROS_DEBUG_NAMED("DfromGoal", "Distance from Goal Way Point  = %f",goalWP_dis);
	   goalWP_mutex.lock();
		scenario_goalWP_min_dis = std::min( scenario_goalWP_min_dis , goalWP_dis);
	   goalWP_mutex.unlock();
}



// grades publishing
std_msgs::Float32MultiArray greads_array;
std_msgs::Bool reset_flag;
ros::Publisher greades_pub_;
ros::Publisher reset_pub_;
void grades_publishing(const ros::TimerEvent&)
{
  	greads_array.data.clear();

	collision_mutex.lock();
	rollover_mutex.lock();
	goalWP_mutex.lock();

    	greads_array.data.push_back(scenario_obj_min_dist);
  		greads_array.data.push_back(scenario_roll_max_ang);
  		greads_array.data.push_back(scenario_pitch_max_ang);

  		greads_array.data.push_back(scenario_goalWP_min_dis);

  		greades_pub_.publish(greads_array);

  		if (  (scenario_obj_min_dist <= MIN_ALLOWED_OBS_DIST)  || ( scenario_roll_max_ang >= MAX_ALLOWED_ROLL_ANG) || (scenario_pitch_max_ang >= MAX_ALLOWED_PITCH_ANG)  )
			{
			reset_flag.data = true;
			//ROS_INFO(" vreset_flag = %d " , reset_flag.data);
			}
		   reset_pub_.publish(reset_flag);

   collision_mutex.unlock();
   rollover_mutex.unlock();
   goalWP_mutex.unlock();
}

/**
 * Loads fcl_models of world objects into a map of <name,world_object_model>
 */
void load_obstacles_models()
{
    obs_models_map = new std::map<std::string ,BVHModel<RSS> *>;
    std::string resours_file = sfv->get_ResourceFile();
    std::string obj_dir_path= ResourceHandler::getInstance(resours_file).getWorldModelsFolderURL();
    std::ofstream mf;mf.open ("graders_obstacles_path.txt"); // Created in ~/.ros
	   int id=1;
	   while(ResourceHandler::getInstance(resours_file).getObjectById(id) != "")
	   {
		   std::string obj_name = ResourceHandler::getInstance(resours_file).getObjectById(id);
		   std::string obj_path = obj_dir_path + "/" + obj_name + "/models/" + obj_name + ".obj";

		  // std::cout << " obj_path = " << obj_path << std::endl;
		  // ROS_DEBUG_NAMED("Obj_Path", "Obj_Path  = %s" , obj_path.c_str());
			
	  		mf << obj_path.c_str();
			mf << "\n";
		        
		   // ROS_INFO(" obj_path = %s " , obj_path.c_str());
			//PATH = obj_path.c_str();

			
		   std::vector<Vec3f> o_ver;
		   std::vector<Triangle> o_tri;
		   loadOBJFile( obj_path.c_str() , o_ver, o_tri);

		   BVHModel<RSS> * obj_model = new BVHModel<RSS>();
		   obj_model->beginModel();
		   obj_model->addSubModel(o_ver, o_tri);
		   obj_model->endModel();

		   std::pair<std::string ,BVHModel<RSS> *> pair_name_obj =  std::pair<std::string ,BVHModel<RSS> *>( obj_name , obj_model);
		   obs_models_map->insert(pair_name_obj);

		   id++;
	   }
   mf.close();
}

/**
 * Loads fcl_models of robot parts in to a map of <name,robot_part_model>
 */
void load_robot_models()
{
	robot_models_map = new std::map<std::string ,BVHModel<RSS> *>;

	std::string resours_file = sfv->get_ResourceFile();
	robot_name = ResourceHandler::getInstance(resours_file).getRobotName();

	std::string robot_model_folder_url = ResourceHandler::getInstance(resours_file).getRobotModelsFolderURL();
	std::string robot_model_url = robot_model_folder_url + "/" +  ResourceHandler::getInstance(resours_file).getRobotPlatformName() + "/model.sdf";
        std::ofstream mf;mf.open ("graders_robot_path.txt"); // Created in ~/.ros
	//std::cout << "robot_model_url = " << robot_model_url << std::endl;
		  	mf << robot_model_url.c_str();
			mf << "\n";
	sdf::SDFPtr sdfPtr(new sdf::SDF());
	init(sdfPtr);
	sdf::readFile(robot_model_url,sdfPtr);
	sdf::ElementPtr sdfModelPtr=sdfPtr->root->GetElement("model");
	sdf::ElementPtr sdfUriPtr ;
	   for (sdf::ElementPtr sdfLinkPtr=sdfModelPtr->GetElement("link"); sdfLinkPtr ; sdfLinkPtr=sdfLinkPtr->GetNextElement("link"))
	   {
		   for (sdf::ElementPtr sdfVisualPtr=sdfLinkPtr->GetElement("visual"); sdfVisualPtr; sdfVisualPtr=sdfVisualPtr->GetNextElement("visual"))
		   {
			   if (sdfVisualPtr->GetElement("geometry")->HasElement("mesh"))
			   {
		   sdfUriPtr=sdfVisualPtr->GetElement("geometry")->GetElement("mesh")->GetElement("uri");
		   std::string part_obj_uri = sdfUriPtr->GetValue()->GetAsString();

		   unsigned found1 = part_obj_uri.find_last_of("/");
		   unsigned found2 = part_obj_uri.find_last_of(".");
		   std::string part_name = part_obj_uri.substr(found1+1,(found2-found1)-1);

		   part_obj_uri = robot_model_folder_url + "/" + part_obj_uri.erase( 0 , 8);
		   part_obj_uri = part_obj_uri.erase(part_obj_uri.length()-3 , part_obj_uri.length() ) + "obj";

		  	mf << part_name.c_str();
			mf << "\n";
		   std::vector<Vec3f> p_ver;
		   std::vector<Triangle> p_tri;
		   loadOBJFile( part_obj_uri.c_str() , p_ver, p_tri);

		   BVHModel<RSS> * part_model = new BVHModel<RSS>();
		   part_model->beginModel();
		   part_model->addSubModel(p_ver, p_tri);
		   part_model->endModel();

		   std::pair<std::string ,BVHModel<RSS> *> pair_partName_obj =  std::pair<std::string ,BVHModel<RSS> *>( part_name , part_model);
		   robot_models_map->insert(pair_partName_obj);
			   }
			   else
			   {
				   std::cout << "Grader Warning !!! : part_name = " << sdfVisualPtr->GetAttribute("name")->GetAsString() << " has no mesh " << std::endl;
			   }
		   }
	   }
	mf.close();
}

void printUsage()
{
	std::cout << "Two variables required SFV_ws_folder and SFV_file_url" << std::endl;
}



int main(int argc, char **argv)
{
	/*
	if(argc!=3)
		{
		printUsage();
		return 0;
		}
*/

   ros::init(argc, argv, "greader_node");

   std::string SFV_ws_folder = argv[1];       // "/home/userws3/dany_ws/src/Simulation/srvss/src/SRVSS/scenarios/scenario_1";
   std::string SFV_file_url = argv[2];        // "/home/userws3/dany_ws/src/Simulation/srvss/src/SRVSS/scenarios/scenario_1/test.SFV";
   sfv = new SFV(SFV_file_url,SFV_ws_folder);

   load_obstacles_models();
   load_robot_models();

   ros::NodeHandle n;

   tf::TransformListener listener(ros::Duration(1),true);
   listener_ptr = &listener;

   gazebo_GetModelState_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

   reset_flag.data = false;

   ros::Timer collision_grader_timer = n.createTimer(ros::Duration(0.1), collision_grader);
   ros::Timer rollover_grader_timer = n.createTimer(ros::Duration(0.1), rollover_grader);

   ros::Timer goalWPdis_grader_timer = n.createTimer(ros::Duration(0.2), distance_to_goalWP_grader);

   ros::Timer grades_publishing_timer = n.createTimer(ros::Duration(0.1), grades_publishing);


   greades_pub_ = n.advertise<std_msgs::Float32MultiArray>("/srvss/grades", 100);
   reset_pub_ = n.advertise<std_msgs::Bool>("/srvss/scenario_reset", 100);


   ros::spin();

   return 0;
}

