#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include "sensor_msgs/JointState.h"

#include <cmath>      // std::abs()

//#include "rate2effort/WheelsRate_msg.h"
#include <sstream>
#include <iostream>
#include <fstream>


  ros::Publisher front_left_pub_;
  ros::Publisher front_right_pub_;
  ros::Publisher back_left_pub_;
  ros::Publisher back_right_pub_;

//  ros::Publisher supporter_pub_;
//  ros::Publisher loader_pub_;

  float rot_vel_limit = 2.5;
  float rot_pow_limit = 15000;

  float BL_rot_vel = 0;
  float BR_rot_vel = 0;
  float FL_rot_vel = 0;
  float FR_rot_vel = 0;


void wheelsCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	std_msgs::Float64 pub_FL ;
	std_msgs::Float64 pub_FR ;
        std_msgs::Float64 pub_BL ; 
        std_msgs::Float64 pub_BR ;    
	   	
        pub_FL.data = 0.0;
        pub_FR.data = 0.0;
        pub_BL.data = 0.0;
        pub_BR.data = 0.0;

        float left_com = 0.0;
        float right_com = 0.0;
        if (std::abs(msg->linear.x) + std::abs(msg->angular.x) > 0 ) {
          left_com =   (msg->linear.x + msg->angular.x )/(std::abs(msg->linear.x) + std::abs(msg->angular.x)+0.01) * rot_pow_limit;
	  right_com =  (msg->linear.x - msg->angular.x )/(std::abs(msg->linear.x) + std::abs(msg->angular.x)+0.01) * rot_pow_limit; 
                }

        float FL_mom_limit = 0.0;
        float BL_mom_limit = 0.0;
        float FR_mom_limit = 0.0;
        float BR_mom_limit = 0.0;


// Front Left   
     
        int FL_com_der = 1;
        if (FL_rot_vel*left_com < 0) {
           FL_com_der = -1; 
             }
   
   //     float FL_com_derection =  FL_rot_vel*left_com;
  


        if  ( (std::abs(FL_rot_vel) < 0.2) || (FL_com_der < 0) ) {
            FL_mom_limit = rot_pow_limit;
	     }
        else {    
            FL_mom_limit = std::min(rot_pow_limit , (float)(std::abs(rot_pow_limit/FL_rot_vel)) );
             }
             
        if  ((std::abs(FL_rot_vel) <= rot_vel_limit ) || (FL_com_der < 0) ){
               if (std::abs(left_com) < FL_mom_limit ) { 
	            pub_FL.data = left_com;  
                    }
	       else {
	            pub_FL.data = FL_mom_limit * (left_com/(std::abs(left_com)+0.01) );  
		    }
             }
        front_left_pub_.publish(pub_FL);


// Back Left      
        
        int BL_com_der = 1;
        if (BL_rot_vel*left_com < 0) {
           BL_com_der = -1; 
             }

       // float BL_com_derection =  BL_rot_vel*left_com; 

        if  ( (std::abs(BL_rot_vel) < 0.2) || (BL_com_der < 0) ){
            BL_mom_limit = rot_pow_limit;
             }
        else {    
            BL_mom_limit = std::min(rot_pow_limit , (float)(std::abs(rot_pow_limit/BL_rot_vel)) ) ;
             }

         if ( (std::abs(BL_rot_vel) <= rot_vel_limit ) || (BL_com_der < 0) ) { 
             if (std::abs(left_com) < BL_mom_limit ) { 
	          pub_BL.data = left_com;  
                  }
	     else {
	          pub_BL.data = BL_mom_limit * (left_com/(std::abs(left_com)+0.01));  
		  }
              }
         back_left_pub_.publish(pub_BL);

// Front Right   

        int FR_com_der = 1;
        if (FR_rot_vel*right_com < 0) {
           FR_com_der = -1; 
             }

     //   float FR_com_derection =  FR_rot_vel*right_com;

        if  ( (std::abs(FR_rot_vel) < 0.2) || (FR_com_der < 0) ) {
            FR_mom_limit = rot_pow_limit;
             }
        else {    
            FR_mom_limit = std::min(rot_pow_limit , (float)(std::abs(rot_pow_limit/FR_rot_vel)) );
             }

        if ( (std::abs(FR_rot_vel) <= rot_vel_limit ) || (FR_com_der < 0) ) { 
             if (std::abs(right_com) < FR_mom_limit ) { 
	          pub_FR.data = right_com;  
                  }
	     else {
	          pub_FR.data = FR_mom_limit * (right_com/(std::abs(right_com)+0.01) );  
		  }
               }
           front_right_pub_.publish(pub_FR);


// Back Right

        int BR_com_der = 1;
        if (BR_rot_vel*right_com < 0) {
           BR_com_der = -1; 
	       }
       
        if  ( (std::abs(BR_rot_vel) < 0.2) || (BR_com_der < 0) ){
            BR_mom_limit = rot_pow_limit;
             }
        else {    
            BR_mom_limit = std::min(rot_pow_limit , (float)(std::abs(rot_pow_limit/BR_rot_vel)) );
             }

        if ( (std::abs(BR_rot_vel) <= rot_vel_limit ) || (BR_com_der < 0) ) { 
             if (std::abs(right_com) < BR_mom_limit ) { 
	          pub_BR.data = right_com;  
                   }
	     else {
	          pub_BR.data = BR_mom_limit * (right_com/(std::abs(right_com)+0.01) );  
		  }
               }
           back_right_pub_.publish(pub_BR); 

/*
     ROS_INFO(" ------ " );
     ROS_INFO(" left_com = %f ,  right_com = %f ",  left_com ,  right_com );
     ROS_INFO(" FL_com_der = %d,   BL_com_der = %d,   FR_com_der = %d,   BR_com_der = %d ",   FL_com_der,   BL_com_der,   FR_com_der,   BR_com_der);
     ROS_INFO(" FL_mom_limit = %f, BL_mom_limit = %f, FR_mom_limit = %f, BR_mom_limit = %f ", FL_mom_limit, BL_mom_limit, FR_mom_limit, BR_mom_limit );
     ROS_INFO(" FL_com = %f,       BL_com = %f,       FR_com = %f,       BR_com = %f ",       pub_FL.data , pub_BL.data , pub_FR.data,  pub_BR.data );
*/

    ros::Rate rate(100);
    rate.sleep();
}


void joint_statesCallback(sensor_msgs::JointState msg)
{	
      BL_rot_vel =   msg.velocity[0];
      BR_rot_vel =   msg.velocity[1];
      FL_rot_vel =   msg.velocity[2];
      FR_rot_vel =   msg.velocity[3];
      
 //   ROS_INFO(" FL_rot_vel = %f , BL_rot_vel = %f , FR_rot_vel = %f , BR_rot_vel = %f ", FL_rot_vel , BL_rot_vel , FR_rot_vel , BR_rot_vel );

	ros::Rate rate(100);
    	rate.sleep();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rate2effort");

  ros::NodeHandle n;

  front_left_pub_ = n.advertise<std_msgs::Float64>("/srvss_bobcat/front_left_wheel_velocity_controller/command", 100);
  front_right_pub_ = n.advertise<std_msgs::Float64>("/srvss_bobcat/front_right_wheel_velocity_controller/command", 100);
  back_left_pub_ = n.advertise<std_msgs::Float64>("/srvss_bobcat/back_left_wheel_velocity_controller/command", 100);
  back_right_pub_ = n.advertise<std_msgs::Float64>("/srvss_bobcat/back_right_wheel_velocity_controller/command", 100);


//  supporter_pub_ = n.advertise<std_msgs::Float64>("/bobcat/supporter_position_controller/command", 100);
//  loader_pub_ = n.advertise<std_msgs::Float64>("/bobcat/loader_position_controller/command", 100);

    ros::Subscriber twist_sub_ = n.subscribe("/wheelsrate", 100, wheelsCallback );   
    ros::Subscriber JointStates_sub_ = n.subscribe("/srvss_bobcat/joint_states", 100, joint_statesCallback );

//  ros::Subscriber arm_sub_ = n.subscribe("/armrate", 1000, armCallback );

  ros::spin();

  return 0;
}
