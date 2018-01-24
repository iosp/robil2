#include "ros/ros.h"

#include <iostream>
#include <string>     // std::string, std::stof
#include <sstream>
#include <stdio.h>
#include <array>

#include "std_srvs/Empty.h"

#include "std_msgs/String.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/LinkState.h"

#include <vector>

ros::ServiceClient pause_physics_updates_client;

ros::ServiceClient links_state_client;
ros::Subscriber links_states_sub;

std::vector <gazebo_msgs::LinkState *> * previous_links_state;

bool init = true;

void remmember_previous_link_state(gazebo_msgs::LinkStates::ConstPtr  new_link_states_msg)
{
	previous_links_state = new std::vector<gazebo_msgs::LinkState *>;
	gazebo_msgs::LinkState * link;

	int i=0;
	for (std::string link_name_it : (new_link_states_msg->name) )
	{
		link = new gazebo_msgs::LinkState();

		link->link_name = new_link_states_msg->name[i].data();
		link->pose = new_link_states_msg->pose[i];
		//link->twist = new_link_states_msg->twist[i];

		previous_links_state->push_back(link);
		i++;
	}
}


void link_states_callback(gazebo_msgs::LinkStates::ConstPtr  new_link_states_msg)
{

	if (init) { remmember_previous_link_state(new_link_states_msg)  ; init = false; }

	gazebo_msgs::LinkState next_link_state;

	int i=0;
	for (std::string link_name_it : (new_link_states_msg->name) )
	{
		if ( ( new_link_states_msg->pose[i].position.x != previous_links_state->at(i)->pose.position.x) ||
			 ( new_link_states_msg->pose[i].position.y != previous_links_state->at(i)->pose.position.y) ||
			 ( new_link_states_msg->pose[i].position.z != previous_links_state->at(i)->pose.position.z) ||
			 ( new_link_states_msg->pose[i].orientation.x != previous_links_state->at(i)->pose.orientation.x) ||
			 ( new_link_states_msg->pose[i].orientation.y != previous_links_state->at(i)->pose.orientation.y) ||
			 ( new_link_states_msg->pose[i].orientation.z != previous_links_state->at(i)->pose.orientation.z) ||
			 ( new_link_states_msg->pose[i].orientation.w != previous_links_state->at(i)->pose.orientation.w) )
				{
				next_link_state.link_name=new_link_states_msg->name[i].data();
				next_link_state.pose = new_link_states_msg->pose[i];
				//	next_link_state.twist = new_link_states_msg->twist[i];

				gazebo_msgs::SetLinkState set_link_state_req;
				set_link_state_req.request.link_state = next_link_state;

				if (! links_state_client.call(set_link_state_req))
					{
					ROS_INFO(" Failed to call service !!!");
					}
				}
		i++;
	}
	 remmember_previous_link_state(new_link_states_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "PeplayPlay");

  ros::NodeHandle n;
  pause_physics_updates_client = n.serviceClient<std_srvs:: Empty>("/gazebo/pause_physics");

  links_state_client = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
  links_states_sub = n.subscribe("/gazebo/link_states", 100, link_states_callback);

  std_srvs::Empty empty;
  pause_physics_updates_client.call(empty);


  ros::spin();

  return 0;
}
