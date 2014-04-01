#include "ros/ros.h"
#include <tf/tf.h>
#include "math.h"       // atan2()  sqrt()
#include <cmath>      // std::abs()
#include <algorithm>    // std::min
 
#include <iostream>
#include <string>     // std::string, std::stof

#include "std_msgs/String.h"


#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

#include "sensor_msgs/LaserScan.h" 

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

#include "geometry_msgs/Pose.h"

#include <sstream>
#include <stdlib.h>

#include <boost/thread/mutex.hpp>

 
     int wp_num = 1;
     const int max_wp_num = 10;
     int wp = 0;

     float t_lat[max_wp_num];   // path lat array 
     float t_lon[max_wp_num];   // path lon array


// receiving the WP command and overrides the existing target WP and WP path
int keybord_com_wp = 0; // flag that tells the wp_driver that a commanded WP was received (and that previous WP can be lost) 
boost::mutex WP_mutex;
void WP_callback(const geometry_msgs::Pose WP_msg)
{    
  WP_mutex.lock(); 
          keybord_com_wp = 1;
        
          t_lat[1] = WP_msg.position.x;
          t_lon[1] = WP_msg.position.y;

          ROS_INFO( " !!! RECEIVED NEW TARGET WP : lat = %f , lon = %f " , WP_msg.position.x , WP_msg.position.y );

  WP_mutex.unlock();
}


// receiving the GPS data and extracting the relevant bobcat lat-lon and centralizing it to (0,0)
float new_bobcat_lat = -1;
float new_bobcat_lon = -1;
boost::mutex GPS_data_mutex;
void GPS_callback(const sensor_msgs::NavSatFix GPS_msg) 
{    
  GPS_data_mutex.lock(); 

      new_bobcat_lat =  (GPS_msg.latitude - 31.2622) * (1/0.000009) ;     // GPS sensor origin (lat = 31.2622, lon = 34.803611) - BenGuriyon University 
      new_bobcat_lon =  (GPS_msg.longitude - 34.803611) * (1/0.000009) ;  // (1/0.000009) is used for approximate conversion of lat, lon degrees to meters

  GPS_data_mutex.unlock();
}


// receiving the IMU data and extracting the bobcat azimuth from the Quaternion 
float new_bobcat_azi = -1;
float new_bobcat_ang_vel;
boost::mutex IMU_data_mutex;

void IMU_callback(const sensor_msgs::Imu IMU_msg)
{   
  IMU_data_mutex.lock(); 
    float z = IMU_msg.orientation.z;
    float w = IMU_msg.orientation.w;
     
    float azi = 0;
    if ( z > 0 )
      { new_bobcat_azi = 2*acos(w); }
    else
      { new_bobcat_azi = -2*acos(w); }
 
    new_bobcat_ang_vel = IMU_msg.angular_velocity.z;  
  IMU_data_mutex.unlock();
}



// receiving the SICK data and extracting the ranges relevant for the obstacle avoidance, also counts the number of readings in each buffer section 
int samp_num = 150;
const int buffer_sections = 3;
int new_SICK_buffer_data[buffer_sections];    // bobcat front buffer
const int  buffer_threshold = 1500;       // buffer threshold the number of samples in order a certain buffer section will show as active
float obs_dist = 10;

boost::mutex SICK_buffer_mutex;
void SICK_callback(const sensor_msgs::LaserScan::ConstPtr& SICK_msg) 
{    
  SICK_buffer_mutex.lock(); 
     for (int i=0 ; i<samp_num ; i++)
       { 
         float range =  SICK_msg->ranges[(760-samp_num)/2+i];      // 760 is the total number of range readings in each SICK msg. the relevant samples are are taken from the middle.
         int buffer_section = (int)(buffer_sections * i/samp_num);

         if  ( (range <= obs_dist) && (  new_SICK_buffer_data[buffer_section] < 2* buffer_threshold ) )
             {                  
              new_SICK_buffer_data[buffer_section] = new_SICK_buffer_data[buffer_section] + 3;   // accumulation of readings         
             }            

         if  ( new_SICK_buffer_data[buffer_section] > 0 ) 
             {
              new_SICK_buffer_data[buffer_section] = new_SICK_buffer_data[buffer_section] - 1;   // fading out of readings
             } 
        }
  SICK_buffer_mutex.unlock();
}


//initiation of the navigation variables based on the first received GPS and INS data 
  
     float bobcat_lat;
     float bobcat_lon;
     float bobcat_lin_vel;

     float previous_lat ; 
     float previous_lon ;
     float previous_time;
     float previous_t_azi;
     int jt = 0;   // counter of rotation of the t_azi, needed in order to prevent jumps near 3.14 
     float previous_bobcat_azi;
     int jb = 0;   // counter of rotation of the bobcat_azi, needed in order to prevent jumps near 3.14 
 
     float current_lat_t;
     float current_lon_t;

     float lat_error;
     float lon_error;
     float dist_error;   
     float azi_error;

     float I_dist_error = 0;  // integral of dist_error -  not in use for now
     float I_azi_error = 0;   // integral of azi_error - not in use for now

     float bobcat_init_lat = -1;   // -1 signifies that no dlat data was received 
     float bobcat_init_lon = -1;   // -1 signifies that no lon data was received 
     float bobcat_init_azi = -1;   // -1 signifies that no azi data was received 

     int nav_data_ready = 0; // a flag that alows the work of the wp_driver and obstacle_avoidance functions only after GPS and INS data was received

int init_nav_data()  
{
   while (nav_data_ready != 1)  // looping until the GPS and IMU data are available
    {
     GPS_data_mutex.lock();            
       bobcat_init_lat = new_bobcat_lat;  
       bobcat_init_lon = new_bobcat_lon;
     GPS_data_mutex.unlock(); 

     IMU_data_mutex.lock(); 
       bobcat_init_azi = new_bobcat_azi; 
     IMU_data_mutex.unlock();

     WP_mutex.lock(); 
      t_lat[0] = bobcat_init_lat;   // the first wp is the initial position of the bobcat  
      t_lon[0] = bobcat_init_lon; 

      current_lat_t =  t_lat[0];   
      current_lon_t =  t_lon[0];
     WP_mutex.unlock(); 

     bobcat_lat =  bobcat_init_lat;
     bobcat_lon =  bobcat_init_lon;
     bobcat_lin_vel = 0;
     previous_time = ros::Time::now().toSec();

     lat_error = current_lat_t - bobcat_init_lat;
     lon_error = current_lon_t - bobcat_init_lon;
     dist_error = sqrt(lat_error*lat_error + lon_error*lon_error);

     azi_error = 0; 
    
     previous_bobcat_azi = bobcat_init_azi;
     previous_t_azi = atan2(lon_error,lat_error);

      if ( (bobcat_init_lat != -1) && (bobcat_init_lon != -1)  && (bobcat_init_azi != -1) )       
          nav_data_ready = 1;   // a flag that alows the work of the wp_driver and obstacle_avoidance functions 
       
    ROS_INFO( " nav_data_ready = %d " , nav_data_ready );

    ros::spinOnce();
     }
    return 0;
}



// wp_driver calculating the required bobcat acceleration and stering commands in order to get to a WP, and publishing them on ROS topic. 

     ros::Publisher wheelsrate_pub;
     
     float max_vel = 2;
     float WP_passing_dist = 1;  // the distance that the bobcat required to approach a WP in order to claim that it reached it, and go for the next (if next WP exist).





     int bypass_on = 0;   // flag that shows that the a current WP is a bypass WP (and not a path WP)

void wp_driver(const ros::TimerEvent& )
{  
     if (nav_data_ready == 0)   // waiting for the GPS and IMU data to be available
       return; 


     // in case that a command WP was received the initial path will be lost. the new path will have only the new last commanded WP
     WP_mutex.lock();         
        if ( keybord_com_wp == 1 )   
           {
              wp = 1;       
              wp_num = 1;

              current_lat_t = t_lat[wp];
              current_lon_t = t_lon[wp];

              keybord_com_wp = 0;
           }
     WP_mutex.unlock(); 


     //receiving data from GPS and INS 
     GPS_data_mutex.lock();            
       bobcat_lat = new_bobcat_lat;
       bobcat_lon = new_bobcat_lon;
     GPS_data_mutex.unlock(); 

     IMU_data_mutex.lock(); 
       float bobcat_ang_vel = new_bobcat_ang_vel;
     IMU_data_mutex.unlock();

     // calculation of lat-lon distances to the target WP
     lat_error = current_lat_t - bobcat_lat;
     lon_error = current_lon_t - bobcat_lon;
     
     // calculation of time interval
     float current_time = ros::Time::now().toSec();
     float time_interval = current_time - previous_time; 
     previous_time = current_time;

     // geting and adjusting the bobcat azimut ;
     IMU_data_mutex.lock(); 
       float bobcat_azi = new_bobcat_azi; 
     IMU_data_mutex.unlock();
   
     if ( (bobcat_azi + jb*2*3.14) - previous_bobcat_azi > 1 )
             { jb = jb-1; }
     else if  ( (bobcat_azi + jb*2*3.14) - previous_bobcat_azi < -1 )
             { jb = jb+1; }
     bobcat_azi = bobcat_azi + jb*2*3.14;
     previous_bobcat_azi = bobcat_azi ;
  
     // calculation and adjusting the target azimuth 
     float t_azi = atan2(lon_error,lat_error);
     if  (((t_azi+jt*2*3.14) - previous_t_azi) > 1) 
            { jt = jt-1; }
     else if (((t_azi+jt*2*3.14) - previous_t_azi) < -1)
            { jt = jt+1; }
       t_azi = t_azi + jt*2*3.14;
       
     float temp_azi_error = t_azi - bobcat_azi;
     if ( std::abs(temp_azi_error) > (2*3.14 - std::abs(temp_azi_error)) ) 
           {  
         if ( temp_azi_error <= 0) 
               { t_azi = t_azi + 2*3.14;}
         else
               { t_azi = t_azi - 2*3.14; }
           } 

     // calculation of the bobcat velocity azimuth 
     float bobcat_lat_diff = bobcat_lat - previous_lat;
     float bobcat_lon_diff = bobcat_lon - previous_lon;
     float bobcat_vel_azi = atan2( bobcat_lon_diff , bobcat_lat_diff);
    
     // calculation of the bobcat velocity (relative to the bobcat azimuth)
     bobcat_lin_vel =  sqrt ( pow( bobcat_lat_diff/time_interval , 2) + pow( bobcat_lon_diff/time_interval , 2) );        
     float relative_vel_der =  std::abs(bobcat_vel_azi - (bobcat_azi-jb*2*3.14));
     if ( relative_vel_der <= 1.57 ) 
          { bobcat_lin_vel =  sqrt( pow( bobcat_lat_diff/time_interval , 2) + pow( bobcat_lon_diff/time_interval , 2) ); } 
     else 
          { bobcat_lin_vel = - sqrt( pow( bobcat_lat_diff/time_interval , 2) + pow( bobcat_lon_diff/time_interval , 2) ); }     

    
     // distance, azimuth and velocity errors calculation 
           dist_error = sqrt(lat_error*lat_error + lon_error*lon_error);
           azi_error = t_azi - bobcat_azi; 
  
     float t_vel = max_vel * std::min( 1/(3*std::abs(azi_error)+1) , (float)1.0 );
     float vel_error = t_vel - bobcat_lin_vel;
   

     // calculation of the error Integrals (Not in use for now)
     I_dist_error = I_dist_error + dist_error * time_interval;
     I_azi_error = I_azi_error + azi_error * time_interval;

  
     // Run time information
     ROS_INFO(" ------- ");  
     ROS_INFO(" current_time = %f , time_interval = %f" , current_time , time_interval); 
     ROS_INFO(" wp_num = %d , wp = %d " , wp_num , wp);
     ROS_INFO(" current_lat_t = %f , current_lon_t = %f , target_vel = %f , target_azi = %f" , current_lat_t , current_lon_t , t_vel , t_azi);
     ROS_INFO(" bobcat_lat = %f , bobcat_lon = %f , bobcat_lin_vel= %f , bobcat_azi = %f " , bobcat_lat , bobcat_lon , bobcat_lin_vel , bobcat_azi );
     ROS_INFO(" lat_error = %f , lon_error = %f , dist_error = %f , vel_error = %f , azi_error = %f" , lat_error , lon_error , dist_error , vel_error , azi_error );
     ROS_INFO(" bobcat_lin_vel = %f , bobcat_ang_vel = %f ", bobcat_lin_vel , bobcat_ang_vel );
  
    // control loop
     geometry_msgs::Twist twistMsg; 
     if (dist_error < WP_passing_dist)  // reaching a WP that can be a path WP or a baypass WP
          {      
               if ( ((wp+1) <= wp_num) || (bypass_on == 1))
                 { 
                   jt = 0;  jb = 0;  
             
                   if (bypass_on == 1)   // reacing baypass WP
                    { 
                       bypass_on = 0;
                       ROS_INFO("Reache bypass WP");      
                    }
                   else if (bypass_on == 0) // reacing path WP
                    { 
                       wp = wp + 1;
                       ROS_INFO("Reached a wp %d of %d !!! going for to the next..." , wp , wp_num );  
                    }
                    WP_mutex.lock();
                      current_lat_t = t_lat[wp];
                      current_lon_t = t_lon[wp];
                    WP_mutex.unlock(); 
                 }    
              else   // reaching the final path WP
                {
                  ROS_INFO("Reached final WP %d !!! End of mission !!! " , wp );  
                  if (bobcat_lin_vel >= 0.005 ) 
                     { twistMsg.linear.x = -0.0001*bobcat_lin_vel; }
                  else if (bobcat_lin_vel <= -0.005 ) 
                     { twistMsg.linear.x = 0.0001*bobcat_lin_vel; }     
                }                  
          }
      else    // the actual control logic 
          {
            if  (std::abs(vel_error) >= 0.1 )  
               { twistMsg.linear.x = 0.10 * vel_error ; }
            else 
               { twistMsg.linear.x = 0; }

            if ( (std::abs(azi_error) >= 0.1) && ( std::abs(bobcat_ang_vel) < 1 ) ) 
               { twistMsg.angular.x = -0.90 * std::min( std::abs(azi_error) , (float)1.0 ) * (azi_error/std::abs(azi_error))  ;  }
            else 
               { twistMsg.angular.x = 0; }
        
            ROS_INFO(" liniar command = %f , angular command = %f " , twistMsg.linear.x , twistMsg.angular.x );
          }

      wheelsrate_pub.publish(twistMsg);
     
    // data for the next loop
      previous_t_azi = t_azi ;
      previous_lat = bobcat_lat;
      previous_lon = bobcat_lon;
}


// obstacle_avoidance identify if an obstacle is situated in the buffer region and if required calculates a new WP that avoids the obstacle 

bool buf_act[buffer_sections]; // bobcat front buffer
float bypass_dist = 1.5;    // scale factor of bypass distance

void obstacle_avoidance(const ros::TimerEvent&)
{
     if (nav_data_ready == 0)  // waiting for the GPS and IMU data to be available
       return;    

     if (std::abs(azi_error) > 0.3)  // while the bobcat do not facing in the direction of a WP there is no reason to check if there is an obstacle and create unnecessary bypasses 
      return; 


     int buf[buffer_sections];
     SICK_buffer_mutex.lock();           
          for (int k=0 ; k<buffer_sections ; k++) 
              { buf[k] = new_SICK_buffer_data[k]; }
     SICK_buffer_mutex.unlock();
    
      // 
       for (int j=0; j<buffer_sections ; j++)  
             {                                
         if ( buf[j] >  buffer_threshold)
               buf_act[j] = true; 
	 else
               buf_act[j] = false; 
  
         ROS_INFO(" buffer_section = %d , value = %d , buf_act = %d " , j , buf[j] , buf_act[j]);
           }
      
      
      float bypass_dir = 0;   // positive is left
      int begin_bypass = 0;   // flag that tells whether a bypass WP shall be calculated 


      // bypass logic           
         if (   ( (! buf_act[0]) && (! buf_act[1]) && (! buf_act[2]) ) ||    // 0 0 0  Go center => begin_bypass = 0   
                ( (  buf_act[0]) && (! buf_act[1]) && (  buf_act[2]) ) )     // 1 0 1  Go center => begin_bypass = 0
              { bypass_dir = 0; begin_bypass = 0;} 

         if (   ( (  buf_act[0]) && (! buf_act[1]) && (! buf_act[2]) ) ||    // 1 0 0  Go left         
                ( (! buf_act[0]) && (  buf_act[1]) && (! buf_act[2]) ) )     // 0 1 0  Go left
              { bypass_dir = 1; begin_bypass = 1; }  

         if (   ( (  buf_act[0]) && (  buf_act[1]) && (! buf_act[2]) )  )    // 1 1 0  Go left-left         
              { bypass_dir = 2; begin_bypass = 1; } 

         if (   ( (  buf_act[0]) && (  buf_act[1]) && (  buf_act[2]) )  )    // 1 1 1  Go left-left-left         
              { bypass_dir = 3; begin_bypass = 1; }  

         if (   ( (! buf_act[0]) && (  buf_act[1]) && (  buf_act[2]) )  )    // 0 1 1  Go right-right         
              { bypass_dir = -2; begin_bypass = 1; }  

         if (   ( (! buf_act[0]) && (! buf_act[1]) && (  buf_act[2]) )  )    // 0 0 1  Go right         
              { bypass_dir = -1; begin_bypass = 1; }  
            
     
      // if a bypass is needed overrides the calculate new WP and overrides the Current WP 
         if (begin_bypass == 1) 
          {
         // the bypass WP located <bypass_dist*bypass_dir> to the left of the line between the bobcat and the previous WP, and <0.7*obs_dist> infront of the bobcat on that line. 
         float bypass_lat_t = bobcat_lat + lat_error/dist_error * 0.7*obs_dist - lon_error/dist_error * bypass_dist * bypass_dir; 
         float bypass_lon_t = bobcat_lon + lon_error/dist_error * 0.7*obs_dist + lat_error/dist_error * bypass_dist * bypass_dir;   
                             
         current_lat_t = bypass_lat_t;  
         current_lon_t = bypass_lon_t;

         bypass_on = 1;   // raising a flag so the wp_driver function will know that it going for a bypass WP 

         ROS_INFO(" beginning bypass !!! ");
          } 
       
     ROS_INFO(" bypass_on = %d ,  bypass_dir = %f " , bypass_on ,  bypass_dir);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "srvss_wp_driver_node");
 
  if (  ((argc-1) > max_wp_num*2 )  || (  ((argc-1)%2)!=0  )  )
      {
         ROS_ERROR(" wrong input !!! ");
         if  ( ((argc-1)%2)!=0 )   
           { ROS_ERROR(" Not equal lat-log parameters number "); }   
         else if ( (argc-1) > max_wp_num*2 )
           { ROS_ERROR(" To many WP, the max allowed WP number is %d you sent %d " , max_wp_num, (argc-4)/2); }
         else 
           { ROS_ERROR(" It is not clear why... ");}         
     return 1;     
      }

  ros::NodeHandle n;

  ros::Subscriber WP_command = n.subscribe("/WP_command", 100, WP_callback); 

  ros::Subscriber GPS_data = n.subscribe("/SENSORS/GPS", 100, GPS_callback); 
  ros::Subscriber IMU_data = n.subscribe("/SENSORS/IMU", 100, IMU_callback); 
  ros::Subscriber SICK_data = n.subscribe("/front_sick/scan", 100, SICK_callback); 
    
  init_nav_data(); // a loop that waits for reception of GPS and INS data

  wp_num = ((argc-1)/2);   
 
  WP_mutex.lock();
    for (int i=1 ; i<=wp_num ; i++)  
       {   
          t_lat[i] = std::atof(argv[2*i-1]);
          t_lon[i] = std::atof(argv[2*i]);  
       }
  WP_mutex.unlock();
 
  ros::Timer wp_driver_timer = n.createTimer(ros::Duration(0.05), wp_driver);
  ros::Timer obstacle_avoidance_timer = n.createTimer(ros::Duration(0.25), obstacle_avoidance);     
  wheelsrate_pub = n.advertise<geometry_msgs::Twist>("/wheelsrate", 1000);
 
  ros::spin();

  return 0;
}
