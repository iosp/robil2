
#include <sstream>
#include <iostream>

std::string topic_name__platform_angular = "/RQT_paltform_velocity_monitor/angular";
std::string topic_name__platform_linear = "/RQT_paltform_velocity_monitor/linear";



#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <mlc/mlcConfig.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/Config.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


const int MODE_control_prediction=2;
const int MODE_dynamic_angular_velocity=1;

mlc::mlcConfig config;
ros::Publisher p_trajectory_params;
ros::Publisher p_cmd_vel;

void callback(mlc::mlcConfig &config, uint32_t level) {
  ::config = config;
}

dynamic_reconfigure::DoubleParameter double_param(std::string name, double value)
{
	dynamic_reconfigure::DoubleParameter _double_param;
	_double_param.name=name;
	//std::stringstream s; s<<value;
	_double_param.value=value;
	return _double_param;
}

void speed_adapter( double lin_vel, double ang_vel,   double& min_vel_t, double& max_vel_t)
{
	min_vel_t = -0.1;
	max_vel_t = 0.1;

	double lls = config.low_lin_speed;
	double hls = config.hig_lin_speed;
	double llsi = config.low_ang_speed_interval;
	double hlsi = config.hig_ang_speed_interval;

	if( lin_vel < lls ){ min_vel_t = -llsi; max_vel_t = llsi; return; }
	if( lin_vel > hls ){ min_vel_t = -hlsi; max_vel_t = hlsi; return; }

	if( fabs(lls - hls)<0.001 ) hls = 0.0001;
	double k = (hlsi-llsi)/(hls-lls) ;
	double b = llsi - (k*lls);

	double r = k*lin_vel+b;
	min_vel_t = -r;
	max_vel_t = +r;


}


double platform_lin_speed=0;
double platform_ang_speed=0;

double platform_lin_speed_smoothed=0;
double platform_ang_speed_smoothed=0;
double platform_lin_speed_smoothed_coof=0.5;
double platform_ang_speed_smoothed_coof=0.5;

double min_vel_t(0), max_vel_t(0);
geometry_msgs::Twist nmsg;
ros::Time last_twist_update_time;
bool is_check_twist_timeout = false;


void set_ziro_prediction()
{
	ROS_INFO_ONCE("MLC: Set ZIRO prediction between /ROBOT_CENTER and /ROBOT_CENTER_PREDICTION frames");
	  static tf::TransformBroadcaster br;
	  tf::Vector3 location(0,0,0);
	  double heading=0;
	  tf::Transform transform;
	  transform.setOrigin( location );
	  tf::Quaternion q;
	  q.setRPY(0, 0, heading);
	  transform.setRotation(q);
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ROBOT_CENTER", "/ROBOT_CENTER_PREDICTION"));
}

void on_twist_command( const geometry_msgs::TwistConstPtr& msg )
{
	if(config.mode == MODE_control_prediction )
	{
		is_check_twist_timeout = true;
		last_twist_update_time = ros::Time::now();
		if(config.speed_source==2)
		{
//			platform_lin_speed = 1.0;
			platform_lin_speed = msg->linear.x;
			platform_lin_speed_smoothed = (1-platform_lin_speed_smoothed_coof)*platform_lin_speed_smoothed + (platform_lin_speed_smoothed_coof)*platform_lin_speed;

//			platform_ang_speed = M_PI/4.0;
			platform_ang_speed = msg->angular.z;
			platform_ang_speed_smoothed = (1-platform_ang_speed_smoothed_coof)*platform_ang_speed_smoothed + (platform_ang_speed_smoothed_coof)*platform_ang_speed;
		}
	}

	if(config.mode == MODE_dynamic_angular_velocity )
	{
		double _min_vel_t(0), _max_vel_t(0);
		speed_adapter(msg->linear.x, msg->angular.z, _min_vel_t, _max_vel_t);
		if( fabs(min_vel_t-_min_vel_t) < 0.01 == false and fabs(max_vel_t-_max_vel_t)<0.01 )
			return;
		max_vel_t = _max_vel_t;
		min_vel_t = _min_vel_t;

		dynamic_reconfigure::Config trj_params;
		trj_params.doubles.push_back(double_param("max_vel_theta", max_vel_t));
		trj_params.doubles.push_back(double_param("min_vel_theta", min_vel_t));
		p_trajectory_params.publish(trj_params);

		set_ziro_prediction();

		p_cmd_vel.publish(*msg);
	}
	else
	if(config.mode == MODE_control_prediction != 0 )
	{

		double f = config.cmd_vel_smoothing;
		nmsg.linear.x = (1-f)*nmsg.linear.x + (f)*msg->linear.x;
		nmsg.angular.z = (1-f)*nmsg.angular.z + (f)*msg->angular.z;

		p_cmd_vel.publish(nmsg);
	}
	else
	{
		set_ziro_prediction();
		p_cmd_vel.publish(*msg);
	}
}

void prediction()
{
	platform_lin_speed_smoothed_coof = config.platform_speed_smoothing;
	platform_ang_speed_smoothed_coof = config.platform_speed_smoothing;

	  static tf::TransformBroadcaster br;
	  tf::Vector3 location(0,0,0);
	  double heading=0;
	  double time = config.delay_time;
	  int number_of_steps = 10;
	  double step_time = time / (double)number_of_steps;
	  double lin_dist = platform_lin_speed_smoothed*step_time;
	  double ang_dist = platform_ang_speed_smoothed*step_time;
	  //td::cout<<"------- P:"<<platform_lin_speed_smoothed<<", "<<platform_ang_speed_smoothed*57.2958<<" D:"<<lin_dist<<", "<<ang_dist*57.2958<<std::endl;
	  for(int i=0;i<number_of_steps;i++)
	  {
	      
		  tf::Vector3 step(lin_dist*cos(heading), lin_dist*sin(heading), 0);

		  location += step;

		  heading += ang_dist;

	  }

	  //ROS_INFO_STREAM_ONCE( location.x()<<" "<<location.y()<<" "<<heading*57.2958 );
	  ROS_INFO_STREAM_ONCE( "MLC: Send prediction between /ROBOT_CENTER and /ROBOT_CENTER_PREDICTION frames" );

	  tf::Transform transform;
	  transform.setOrigin( location );
	  tf::Quaternion q;
	  q.setRPY(0, 0, heading);
	  transform.setRotation(q);
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/ROBOT_CENTER", "/ROBOT_CENTER_PREDICTION"));
}

tf::Vector3 prev_pose(0,0,0);
tf::Vector3 curr_pose(0,0,0);
double prev_heading=0;
double curr_heading=0;
ros::Time prev_pose_time;
ros::Time curr_pose_time;

tf::Vector3 prev_pose_smoothed(0,0,0);
tf::Vector3 curr_pose_smoothed(0,0,0);
double prev_heading_smoothed=0;
double curr_heading_smoothed=0;


void listen_tf_pose()
{
	static tf::TransformListener listener;

	tf::StampedTransform _robot_pose;
	try{
		listener.lookupTransform("/WORLD", "/ROBOT_CENTER",
				ros::Time(0), _robot_pose);

		curr_pose = tf::Vector3(_robot_pose.getOrigin().x(), _robot_pose.getOrigin().y(), 0);
		double roll, pitch, yaw;
		_robot_pose.getBasis().getRPY(roll, pitch, yaw);
		curr_heading = yaw;
		curr_pose_time = _robot_pose.stamp_;

		double f = config.pose_smoothing;
		curr_pose_smoothed = tf::Vector3( (1-f)*curr_pose_smoothed.x() + f*curr_pose.x(), (1-f)*curr_pose_smoothed.y() + f*curr_pose.y(), 0 );
		curr_heading_smoothed = (1-f)*curr_heading_smoothed + f*curr_heading;

		ros::Duration duration = curr_pose_time - prev_pose_time;
		ros::Duration update_timeout; update_timeout.fromSec(config.tf_pose_update_time);

		if(update_timeout <= duration)
		{
			if(config.speed_source==3)
			{

				tf::Vector3 delta = curr_pose_smoothed - prev_pose_smoothed;
				double length = delta.length();
				double heading=0;
				if(config.tf_heading_source == 1)
				{
					heading = atan2(delta.y(), delta.x());
				}
				if(config.tf_heading_source == 1)
				{
					heading = curr_heading_smoothed - prev_heading_smoothed; heading = atan2(sin(heading),cos(heading)); if(M_PI < heading) heading = 2*M_PI-heading;
				}
				ros::Duration duration = curr_pose_time - prev_pose_time;


				platform_lin_speed = length / duration.toSec();
				platform_lin_speed_smoothed = (1-platform_lin_speed_smoothed_coof)*platform_lin_speed_smoothed + (platform_lin_speed_smoothed_coof)*platform_lin_speed;

				platform_ang_speed = heading / duration.toSec();
				platform_ang_speed_smoothed = (1-platform_ang_speed_smoothed_coof)*platform_ang_speed_smoothed + (platform_ang_speed_smoothed_coof)*platform_ang_speed;
			}


			prev_pose_time = curr_pose_time;
			prev_heading = curr_heading;
			prev_pose = curr_pose;
			prev_heading_smoothed = curr_heading_smoothed;
			prev_pose_smoothed = curr_pose_smoothed;
		}
	}
	catch (tf::TransformException ex){
		ROS_ERROR("MLC: %s",ex.what());
		ros::Duration(1.0).sleep();
	}



}

void on_platform_lin_speed( const std_msgs::Float32::ConstPtr& msg )
{
	if(config.mode != MODE_control_prediction) return;

	//ROS_INFO("on platform lin speed");
	if(config.speed_source==1)
	{
		platform_lin_speed = msg->data;
		platform_lin_speed_smoothed = (1-platform_lin_speed_smoothed_coof)*platform_lin_speed_smoothed + (platform_lin_speed_smoothed_coof)*platform_lin_speed;
	}
	if(config.mode != MODE_control_prediction ) return;
//	listen_tf_pose();
//	prediction();
}
void on_platform_ang_speed( const std_msgs::Float32::ConstPtr& msg )
{
	if(config.mode!=MODE_control_prediction) return;

	//ROS_INFO("on platform ang speed");
	if(config.speed_source==1)
	{
		platform_ang_speed = msg->data;
		platform_ang_speed_smoothed = (1-platform_ang_speed_smoothed_coof)*platform_ang_speed_smoothed + (platform_ang_speed_smoothed_coof)*platform_ang_speed;
	}
	if(config.mode != MODE_control_prediction ) return;
	//prediction();
}

void check_twist_timeout()
{
	if(not is_check_twist_timeout)
	{
		set_ziro_prediction();
		return;
	}


	ros::Duration d = ros::Time::now() - last_twist_update_time;
	if(d.toSec() > 0.5)
	{
		geometry_msgs::Twist stop;
		p_cmd_vel.publish(stop);
		is_check_twist_timeout = false;
		return;
	}

	//listen_tf_pose();
	prediction();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "midle_level_control");

  dynamic_reconfigure::Server<mlc::mlcConfig> server;
  dynamic_reconfigure::Server<mlc::mlcConfig>::CallbackType f;
  
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  
  ros::NodeHandle node;
  ros::Subscriber s_speed_command = node.subscribe("/cmd_vel_orig", 1, on_twist_command);
  p_trajectory_params = node.advertise<dynamic_reconfigure::Config>("/move_base/TrajectoryPlannerROS/parameter_updates", 1);
  p_cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
  
  ros::Subscriber s_platform_lin = node.subscribe(topic_name__platform_linear, 1, on_platform_lin_speed);
  ros::Subscriber s_platform_ang = node.subscribe(topic_name__platform_angular, 1, on_platform_ang_speed);
  
  ROS_INFO("MLC: midle_level_control is running...");
  ros::Rate rate(100);
  while(ros::ok())
  {
	  ros::spinOnce();
	  check_twist_timeout();
	  rate.sleep();
  }
  ROS_INFO("MLC: midle_level_control is stopped");
  return 0;
}



