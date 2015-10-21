#include "ros/ros.h"
#include <tf/tf.h>
#include "math.h"       // atan2()  sqrt()
#include <cmath>      // std::abs()
#include <algorithm>    // std::min
 
#include <iostream>
#include <string>     // std::string, std::stof
#include <fstream>


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

     const double PI = 3.14159265359;


     int wp_num = 1;
     const int max_wp_num = 100;
     int wp = 0;

     float t_lat[max_wp_num];   // path lat array 
     float t_lon[max_wp_num];   // path lon array
     float t_vel[max_wp_num];

// receiving the WP command and overrides the existing target WP and WP path
int keybord_com_wp = 0; // flag that tells the wp_driver that a commanded WP was received (and that previous WP can be lost) 
boost::mutex WP_mutex;
void WP_callback(const geometry_msgs::Pose WP_msg)
{    
  WP_mutex.lock(); 
          keybord_com_wp = 1;
        
          t_lat[1] = WP_msg.position.x;
          t_lon[1] = WP_msg.position.y;
          t_vel[1] = WP_msg.position.z;

          ROS_INFO( " !!! RECEIVED NEW TARGET WP : lat = %f , lon = %f , Speed = %f " , WP_msg.position.x , WP_msg.position.y, WP_msg.position.z );

  WP_mutex.unlock();
}


// receiving the GPS data and extracting the relevant bobcat lat-lon and centralizing it to (0,0)
float new_bobcat_lat = -1;
float new_bobcat_lon = -1;
boost::mutex GPS_data_mutex;
void GPS_callback(const sensor_msgs::NavSatFix GPS_msg) 
{    
      sensor_msgs::NavSatFix p1, p2;
      p1.latitude = 31.2622;     // GPS sensor origin (lat = 31.2622, lon = 34.803611) - BenGuriyon University
      p1.longitude = 34.803611;

      p2.latitude = GPS_msg.latitude;
      p2.longitude = GPS_msg.longitude;


      double R = 6371 * 1000; //[m]
      double lat1 = p1.latitude*PI/180;
      double lat2 = p2.latitude*PI/180;
      double dLat = lat2 - lat1;
      double dLon = (p2.longitude - p1.longitude)*PI/180;

      double a = sin(dLat/2) * sin(dLat/2) +
                  sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
      double c = 2 * atan2(sqrt(a), sqrt(1-a));
      double d = R*c;

      double y = sin(dLon) * cos(lat2);
      double x = cos(lat1) * sin(lat2) -
            sin(lat1) * cos(lat2) * cos(dLon);
      double brng = atan2(y, x);

   GPS_data_mutex.lock();

        new_bobcat_lat =  cos(brng) * d;
        new_bobcat_lon =  sin(brng) * d;

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




//initiation of the navigation variables based on the first received GPS and INS data 
  
     float bobcat_lat;
     float bobcat_lon;
     float bobcat_lin_vel;

     float previous_lat ; 
     float previous_lon ;
     float previous_time;
     float previous_t_azi;
     int jt = 0;   // counter of rotation of the t_azi, needed in order to prevent jumps near PI
     float previous_bobcat_azi;
     int jb = 0;   // counter of rotation of the bobcat_azi, needed in order to prevent jumps near PI
 
     float current_lat_t;
     float current_lon_t;
     float current_max_vel_t;

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
      t_vel[0] = 0;

      current_lat_t =  t_lat[0];   
      current_lon_t =  t_lon[0];
      current_max_vel_t = t_vel[0];
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


// receiving the SICK data and extracting the ranges relevant for the obstacle avoidance, also counts the number of readings in each buffer section
int samp_num = 150;
const int buffer_sections = 3;
int new_SICK_buffer_data[buffer_sections];    // bobcat front buffer
float new_SICK_obs_dist = 100;
const int  buffer_threshold = 1500;       // buffer threshold the number of samples in order a certain buffer section will show as active
float obs_buff_dist = 10;
float obs_dist=100;

boost::mutex SICK_buffer_mutex;
void SICK_callback(const sensor_msgs::LaserScan::ConstPtr& SICK_msg)
{
  SICK_buffer_mutex.lock();
  int dis_counter=0;
  float dis_sum=0;
     for (int i=0 ; i<samp_num ; i++)
       {
         float range =  SICK_msg->ranges[(760-samp_num)/2+i];      // 760 is the total number of range readings in each SICK msg. the relevant samples are are taken from the middle.
         int buffer_section = (int)(buffer_sections * i/samp_num);

         if  ( (range <= obs_buff_dist) && (range <= dist_error+0.25*obs_buff_dist) && (  new_SICK_buffer_data[buffer_section] < 2* buffer_threshold ) )  // Ignores obstacles farther than obs_buff_dist and behind the WP
             {
              new_SICK_buffer_data[buffer_section] = new_SICK_buffer_data[buffer_section] + 3;   // accumulation of readings
              dis_sum = dis_sum+ range;
              dis_counter = dis_counter + 1;
             }
         if  ( new_SICK_buffer_data[buffer_section] > 0 )
             {
              new_SICK_buffer_data[buffer_section] = new_SICK_buffer_data[buffer_section] - 1;   // fading out of readings
             }
         new_SICK_obs_dist = dis_sum/(dis_counter+1);
        }
  SICK_buffer_mutex.unlock();
}


// wp_driver calculating the required bobcat acceleration and stering commands in order to get to a WP, and publishing them on ROS topic. 

     ros::Publisher wheelsrate_pub;
     
     float max_vel = 2;
     float WP_passing_dist = 1;  // the distance that the bobcat required to approach a WP in order to claim that it reached it, and go for the next (if next WP exist).


     int bypass_on = 0;   // flag that shows that the a current WP is a bypass WP (and not a path WP)
     int bypass_by_jump_to_next_wp = 0;  // flag that shows that the obstacle is located on top the currant WP, the bypass shall be performed by moving to next WP

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
              current_max_vel_t = t_vel[wp];

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
   
     if ( (bobcat_azi + jb*2*PI) - previous_bobcat_azi > 1 )
             { jb = jb-1; }
     else if  ( (bobcat_azi + jb*2*PI) - previous_bobcat_azi < -1 )
             { jb = jb+1; }
     bobcat_azi = bobcat_azi + jb*2*PI;
     previous_bobcat_azi = bobcat_azi ;
  
     // calculation and adjusting the target azimuth 
     float t_azi = atan2(lon_error,lat_error);
     if  (((t_azi+jt*2*PI) - previous_t_azi) > 1)
            { jt = jt-1; }
     else if (((t_azi+jt*2*PI) - previous_t_azi) < -1)
            { jt = jt+1; }
       t_azi = t_azi + jt*2*PI;
       
     float temp_azi_error = t_azi - bobcat_azi;
     if ( std::abs(temp_azi_error) > (2*PI - std::abs(temp_azi_error)) )
           {  
         if ( temp_azi_error <= 0) 
               { t_azi = t_azi + 2*PI;}
         else
               { t_azi = t_azi - 2*PI; }
           } 

     // calculation of the bobcat velocity azimuth 
     float bobcat_lat_diff = bobcat_lat - previous_lat;
     float bobcat_lon_diff = bobcat_lon - previous_lon;
     float bobcat_vel_azi = atan2( bobcat_lon_diff , bobcat_lat_diff);

     // calculation of the bobcat velocity (relative to the bobcat azimuth)
     bobcat_lin_vel =  sqrt ( pow( bobcat_lat_diff/time_interval , 2) + pow( bobcat_lon_diff/time_interval , 2) );        

     float relative_vel_der =  std::abs(bobcat_vel_azi - (bobcat_azi-jb*2*PI));
     if ((bobcat_azi-jb*2*PI) > PI)
     	 { relative_vel_der =  std::abs(bobcat_vel_azi - (bobcat_azi-jb*2*PI - 2*PI)); }
     else if ((bobcat_azi-jb*2*PI) < -PI)
    	 { relative_vel_der =  std::abs(bobcat_vel_azi - (bobcat_azi-jb*2*PI + 2*PI)); }

//     ROS_INFO(" !!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
//     ROS_INFO(" bobcat_vel_azi = %f , (bobcat_azi-jb*2*PI) = %f , jb = %d , relative_vel_der = %f", bobcat_vel_azi , (bobcat_azi-jb*2*PI) , jb, relative_vel_der/PI);

     if ( ( (relative_vel_der/PI) <= 0.5 ) || ( (relative_vel_der/PI) >= 1.5) )
          {
    	 	bobcat_lin_vel = sqrt( pow( bobcat_lat_diff/time_interval , 2) + pow( bobcat_lon_diff/time_interval , 2) );
            ROS_INFO(" !!! + bobcat_lin_vel = %f " , bobcat_lin_vel);
          }
     else 
          {
    	    bobcat_lin_vel = - sqrt( pow( bobcat_lat_diff/time_interval , 2) + pow( bobcat_lon_diff/time_interval , 2) );
            ROS_INFO(" !!! - bobcat_lin_vel = %f " , bobcat_lin_vel);
          }

    
     // distance, azimuth and velocity errors calculation 
           dist_error = sqrt(lat_error*lat_error + lon_error*lon_error);
           azi_error = t_azi - bobcat_azi; 
  
     float adjusted_vel_t = current_max_vel_t * std::min( 1/(3*std::abs(azi_error)+1) , (float)1.0 );
     float vel_error = adjusted_vel_t - bobcat_lin_vel;
   

     // calculation of the error Integrals (Not in use for now)
     I_dist_error = I_dist_error + dist_error * time_interval;
     I_azi_error = I_azi_error + azi_error * time_interval;

  
     // Run time information
     ROS_INFO(" ------- ");  
     ROS_INFO(" current_time = %f , time_interval = %f" , current_time , time_interval); 
     ROS_INFO(" wp_num = %d , wp = %d " , wp_num , wp);
     ROS_INFO(" current_lat_t = %f , current_lon_t = %f, current_max_vel_t = %f , target_adjusted_vel = %f , target_azi = %f" , current_lat_t , current_lon_t , current_max_vel_t ,adjusted_vel_t , t_azi);

     ROS_INFO(" bobcat_lat = %f , bobcat_lon = %f , bobcat_lin_vel= %f , bobcat_vel_azi =%f , bobcat_azi = %f " , bobcat_lat , bobcat_lon , bobcat_lin_vel , bobcat_vel_azi , bobcat_azi);

     ROS_INFO(" lat_error = %f , lon_error = %f , dist_error = %f , vel_error = %f , azi_error = %f" , lat_error , lon_error , dist_error , vel_error , azi_error );
     ROS_INFO(" bobcat_lin_vel = %f , bobcat_ang_vel = %f ", bobcat_lin_vel , bobcat_ang_vel );
     ROS_INFO(" distance to obstacle = %f ", obs_dist );
  
    // control loop
     geometry_msgs::Twist twistMsg; 
     if ( (dist_error < WP_passing_dist) || (bypass_by_jump_to_next_wp == 1) )  // reaching a WP that can be a path WP or a bypass WP
          {      
               if ( ((wp+1) <= wp_num) || (bypass_on == 1))
                 { 
                   jt = 0;  jb = 0;  
             
                   if (bypass_on == 1)   // racing bypass WP
                    { 
                       bypass_on = 0;
                       ROS_INFO("Reache bypass WP");      
                    }
                   else if (bypass_on == 0) // racing path WP
                    { 
                       wp = wp + 1;
                       ROS_INFO("Reached a wp %d of %d !!! going for to the next..." , wp , wp_num );  
                    }

                    WP_mutex.lock();
                      current_lat_t = t_lat[wp];
                      current_lon_t = t_lon[wp];
                      current_max_vel_t = t_vel[wp];
                    WP_mutex.unlock(); 
                	bypass_by_jump_to_next_wp = 0;
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

           // if ( (std::abs(azi_error) >= 0.01) && ( std::abs(bobcat_ang_vel) < 1 ) )
              // { twistMsg.angular.x = -2.00 * std::min( std::abs(azi_error) , (float)1.0 ) * (azi_error/std::abs(azi_error))  ;  }

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
     //float obs_dist;
     SICK_buffer_mutex.lock();           
          for (int k=0 ; k<buffer_sections ; k++) 
              { buf[k] = new_SICK_buffer_data[k]; }

          obs_dist =new_SICK_obs_dist;
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
        	 if ( std::abs(obs_dist - dist_error) < 0.2*obs_buff_dist  )   // the obstacle is on the WP so moving to the next WP
        	 	 {
        		 	 bypass_by_jump_to_next_wp = 1;  // flag that shows that the obstacle is located on top the currant WP, the bypass shall be performed by moving to next WP
        	 	 }
        	 else
        	 	 {
        	 		 // the bypass WP located <bypass_dist*bypass_dir> to the left of the line between the bobcat and the previous WP, and <0.7*obs_buff_dist> infront of the bobcat on that line.
        	 		 float bypass_lat_t = bobcat_lat + lat_error/dist_error * 0.7*obs_buff_dist - lon_error/dist_error * bypass_dist * bypass_dir;
        	 		 float bypass_lon_t = bobcat_lon + lon_error/dist_error * 0.7*obs_buff_dist + lat_error/dist_error * bypass_dist * bypass_dir;
                             
        	 		 current_lat_t = bypass_lat_t;
        	 		 current_lon_t = bypass_lon_t;

        	 		 bypass_on = 1;   // raising a flag so the wp_driver function will know that it going for a bypass WP

        	 		 ROS_INFO(" beginning bypass !!! ");
        	 	 }
         	 }
       
     ROS_INFO(" bypass_on = %d ,  bypass_dir = %f " , bypass_on ,  bypass_dir);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "srvss_wp_driver_node");
 
    if ( (std::string(argv[1]).compare("-path")!=0) && (std::string(argv[1]).compare("-file")!=0) )
    {
    	std::cout << "usage:" <<std:: endl;
    	std::cout <<"(1) -path <wp_1_lat> <wp_1_lon> .... <wp_N_lat> <wp_N_lon> # will perform the in line specified WP path" <<std:: endl;
    	std::cout <<"(2) -file <file_name>  # will perform the WP_path specified in the file " <<std:: endl;
		  return 1;
    }
    else if (std::string(argv[1]).compare("-path")==0)
		{
		std::cout << "!! path !!"<<std::endl;
/*
		  if (  ((argc-2) > max_wp_num*2 )  || (  ((argc-2)%2)!=0  )  )
		  	  {
			  ROS_ERROR(" wrong input !!! ");
			  if  ( ((argc-2)%2)!=0 )
			  { ROS_ERROR(" Not equal lat-log parameters number "); }
			  else if ( (argc-2) > max_wp_num*2 )
			  { ROS_ERROR(" To many WP, the max allowed WP number is %d you sent %d " , max_wp_num, (argc-5)/2); }
			  else
			  { ROS_ERROR(" It is not clear why... ");}
			  return 1;
		  	  }
*/
		  wp_num = ((argc-2)/3);

		  std::cout << " wp_num = " << wp_num << std::endl;
		  std::cout << " argc = " << argc << std::endl;
		  std::cout << " argv[0] = " << argv[0] << std::endl;
		  std::cout << " argv[1] = " << argv[1] << std::endl;
		  std::cout << " argv[2] = " << argv[2] << std::endl;
		  std::cout << " argv[3] = " << argv[3] << std::endl;
		  std::cout << " argv[4] = " << argv[4] << std::endl;

	      for (int i=1 ; i<=wp_num ; i++)
		    {
		      t_lat[i] = std::atof(argv[2+3*(i-1)]);
		      t_lon[i] = std::atof(argv[3+3*(i-1)]);
		      t_vel[i] = std::atof(argv[4+3*(i-1)]);
		    }


		}

  else if(std::string(argv[1]).compare("-file")==0)
		{
		std::cout << "!! file !!"<<std::endl;
		//std::string dir_path = "/home/userws3/srvss/devel/lib/SRVSS/";
		std::string file_path = argv[2];

		std::fstream mfile(file_path.data() ,std::ios_base::in);
		//std::fstream mfile("/home/userws3/srvss/devel/lib/SRVSS/myMission.txt",std::ios_base::in);

		char word[10] = "";
		while( (std::string(word).compare("START")!=0) && !mfile.eof() )
		{
			mfile>>word;
			std::cout << word <<std::endl;
		}

		mfile >> t_lat[0];
		mfile >> t_lon[0];
		t_vel[0] = 0;

		while( (std::string(word).compare("WAYPOINTS")!=0) && !mfile.eof() )
		{    mfile>>word; }

		int i = 1;
		while (!mfile.eof())
		{
			float temp_lat, temp_lon, temp_vel;
			mfile >> temp_lat;
			mfile >> temp_lon;
			mfile >> temp_vel;

			t_lat[i] = temp_lat - t_lat[0];
			t_lon[i] = temp_lon - t_lon[0];
			t_vel[i] = temp_vel;
      	    i++;
		}
		 wp_num = --i;
		}


  ros::NodeHandle n;

  ros::Subscriber WP_command = n.subscribe("/WP_command", 100, WP_callback); 

  ros::Subscriber GPS_data = n.subscribe("/SENSORS/GPS", 100, GPS_callback); 
  ros::Subscriber IMU_data = n.subscribe("/SENSORS/INS", 100, IMU_callback); 
  ros::Subscriber SICK_data = n.subscribe("/front_sick/scan", 100, SICK_callback);
    
  init_nav_data(); // a loop that waits for reception of GPS and INS data

  for(int j=0 ; j<=wp_num ; j++)
   	std::cout << "wp[" << j << "] lat = " << t_lat[j] << " lon = " << t_lon[j] << " vel = " << t_vel[j] << std::endl;

  ros::Timer wp_driver_timer = n.createTimer(ros::Duration(0.05), wp_driver);
  ros::Timer obstacle_avoidance_timer = n.createTimer(ros::Duration(0.25), obstacle_avoidance);
  wheelsrate_pub = n.advertise<geometry_msgs::Twist>("/wheelsrate", 1000);

  ros::spin();

  return 0;
}
