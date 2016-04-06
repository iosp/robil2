
#include <sstream>
#include <iostream>
#include <string>

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* strtof */


#include "std_msgs/String.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

std::vector<tf::StampedTransform*> * ST_vec;


tf::Transform transformBuilder(float x,float y,float z,float Roll,float Pitch,float Yaw)
{
	 tf::Transform transform;
	 transform.setOrigin( tf::Vector3(x, y, z) );

	 tf::Quaternion q;
	 q.setRPY(Roll,Pitch,Yaw);
	 transform.setRotation(q);
	 //  transform.setRotation(tf::Quaternion(Roll,Pitch,Yaw)) // do not work correctly

	 return(transform);
}

void TF_Broadcast_callback(const ros::TimerEvent&)
{
	 static tf::TransformBroadcaster br;
	 std::vector<tf::StampedTransform*>::iterator ST_it ;
	 for (ST_it =  ST_vec->begin() ; ST_it != ST_vec->end()  ;  ST_it++ )
	 {
		 br.sendTransform(**ST_it);
	 }
}



int main(int argc, char **argv)
{
      ros::init(argc, argv, "robil2TF_node");
      ros::NodeHandle n;
      
  //    std::cout << " argc = "<< argc << "(argc-1)%8 = " << (argc-1)%8 << "\n";

      if ((argc-1)%8 != 0) {
    	  std::cout << "Incorrect number of inputs !!! \n" <<
    			       "The correct order :  frame_id, child_frame_id , x,y,z , Roll,Pitch,Yaw" << std::endl;
    	  return(0);
      	  	  	  	  	  	 }

      int tfs = (argc-1)/8;

      ST_vec = new (std::vector<tf::StampedTransform*>);

      char* pEnd;
      for(int i=0; i<tfs; i++)
        {
    	  std::string frame_id = std::string(argv[i*8+1]);
    	  std::string child_frame_id = std::string(argv[i*8+2]);
    	  float x = strtof(argv[i*8+3], &pEnd);
    	  float y = strtof(argv[i*8+4], &pEnd);
    	  float z = strtof(argv[i*8+5], &pEnd);
    	  float Roll = strtof(argv[i*8+6], &pEnd);
    	  float Pitch = strtof(argv[i*8+7], &pEnd);
    	  float Yaw = strtof(argv[i*8+8], &pEnd);


    	  tf::StampedTransform *temp = new tf::StampedTransform(transformBuilder(x,y,z,Roll,Pitch,Yaw), ros::Time::now(), frame_id, child_frame_id);
    	  ST_vec->push_back(temp);

    	  std::cout <<" frame_id = " << frame_id << "      child_frame_id = " << child_frame_id << "\n"
    			  	  " x = "    << x    << " y = "     << y <<     " z = "   << z << "\n"
    			      " Roll = " << Roll << " Pitch = " << Pitch << " Yaw = " << Yaw << "\n\n";

       }

      ros::Timer TF_Broadcast_timer = n.createTimer(ros::Duration(0.01), TF_Broadcast_callback);
      ros::spin();
}
