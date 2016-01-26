#include "ros/ros.h"
#include <iostream> // Strings
#include <image_transport/image_transport.h> // Getting Image from sensor
// OpenCV , CV Bridge
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>
// Gazebo Model 
#include "gazebo_msgs/SetModelState.h" 
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/ModelState.h"
// Math
#include <math.h>       /* sin */
#include <cstdlib>	/* rand */
// Quaternion
#include "tf/transform_listener.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"
#include "fcl/math/transform.h"
// Globals
#define PI 3.14159265

int num = 0; // Image Counter
float W=2.5,H=4.5; // Object Width and Height in meter 
int v_ang=80; // Camera View Angle in degree
int x_roi,y_roi,nx_roi=1200,ny_roi=900,nx=3200,ny=2400,i,j,k,l;
float r,theta,Theta,phi,px,py,x,y,xc,yc;

// Random Float Number Between Interval Function
float RandomFloat(float a, float b) {
    float diff,min,r;
    float random = ((float) rand()) / (float) RAND_MAX;
    if (b>a)
    {
    	diff = b - a;
	min = a;
    }  
    else
    {
      	diff = a - b;
	min = b;
    } 
    r = random * diff;
    return min + r;
}

float RandomInt(float a, float b) {
    int max,min;
    if (b>a)
    {
    	max = b;
	min = a;
    }  
    else
    {
      	max = a;
	min = b;
    } 
    return min + (rand() % (int)(max - min) + 1);
}


 //   utility function for creating fcl Quanterion from Roll, Pitch and Yaw
 
fcl::Quaternion3f Quanterion3f_from_RPY(float Roll,float Pitch,float Yaw)
{
	tf::Matrix3x3 obs_mat;
	obs_mat.setEulerYPR(Yaw,Pitch,Roll);

	tf::Quaternion q_tf;
	obs_mat.getRotation(q_tf);

	fcl::Quaternion3f q(q_tf.getW(),q_tf.getX(),q_tf.getY(),q_tf.getZ());
	return(q);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  x_roi=RandomInt((int)((nx+px)/2)-nx_roi,x),y_roi=RandomInt((int)((ny+py)/2)-ny_roi,y);
  xc = x-x_roi;
  yc = y-y_roi;
  cv::Rect roi(x_roi,y_roi,nx_roi,ny_roi);
  cv::Mat im_roi; // Region of Interest
  num = num +1;
  std::ostringstream picname,cr; // Image Name String
  picname << "img" << num << "_x_"<< x <<"_y_" << y << "_px_"<< px <<"_py_" << py << "_3200x2400.png";
  cr << "crop_im" << num << "_x_"<< xc <<"_y_" << yc << "_px_"<< px <<"_py_" << py << "_1200x900.png";
  try
  {
    //ROS_DEBUG("Picture Num = %d ,X = %d,Y = %d",num,x,y); // Debug Sampling
	//cv::imshow("view",cv_bridge::toCvShare(msg, "bgr8")->image); // saving image
    cv::imwrite(picname.str(), cv_bridge::toCvShare(msg, "bgr8")->image); // saving image

    im_roi=cv::imread(picname.str()); //cropping image
      im_roi=im_roi(roi); 
    i=(int)xc;j=(int)yc;k=(int)(xc+px);l=(int)(yc+py);
    cv::rectangle(im_roi,cv::Point(i,j),cv::Point(k,l),cv::Scalar(0,0,255),1);
    cv::imwrite(cr.str(), im_roi); // end cropping and saving
	cv::imshow("view",im_roi); // showing image
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}




int main(int argc, char** argv)
   {
	srand(time(NULL)); //initialize random per run
	ros::init(argc, argv, "visual_odometry_data_base");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("SENSORS/CAM/TEST", 1, imageCallback); // Camera Subscriber

	ros::ServiceClient  pp_c = n.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
	gazebo_msgs::GetPhysicsProperties getphysicsproperties;
	pp_c.call(getphysicsproperties);//getphysicsproperties now holds the current properties
	//create your desired model state and call the /gazebo/set_model_state service here
	gazebo_msgs::GetModelState gcs,gos; //gcs - Get Camera State , gos - Get Obstacle State
	gcs.request.model_name="flea3";
	gos.request.model_name="object0";
	ros::ServiceClient gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gms_c.call(gcs);//gcs now holds the current state
	gms_c.call(gos);
  
	//double r = 10.0,theta=PI/6,phi=0;
	//float r,theta,phi,px,py,x1,y1;
	fcl::Quaternion3f q;
	r = RandomFloat(10,40);Theta=-PI/2;theta=RandomFloat((0*PI)/180,(PI*30)/180);phi=RandomFloat(0,2*PI);
       // theta=Theta;
	px = (W*nx)/(r*v_ang*(PI/180));
        py = (H*ny)/(r*v_ang*(PI/180));
	x = ((nx-px)/2);
	y = ((ny-py)/2);
// ROS_INFO("Hello MAIN : \n               R = %f     Px,Py(%f,%f)     x,y(%f , %f)",r*sin(theta),px,py,x,y);
     
	geometry_msgs::Twist twist;
	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = 0.0;
	geometry_msgs::Pose pose;

	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	gazebo_msgs::SetModelState setmodelstate;
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name ="flea3";
	modelstate.pose = pose;
	modelstate.twist = twist;
	setmodelstate.request.model_state=modelstate;
        
	while(ros::ok())
	{
		if (!(num%10))
		{
			r = RandomFloat(10,40);theta=RandomFloat((0*PI)/180,(PI*30)/180);phi=RandomFloat(0,2*PI);
		}

		// rotating the camera around the object
		pose.position.x = gos.response.pose.position.x + r*sin(Theta+theta)*cos(phi);
		pose.position.y = gos.response.pose.position.y + r*sin(Theta+theta)*sin(phi);
		pose.position.z = gos.response.pose.position.z + r*cos(Theta+theta) + (H)*0.40;
	
		// rotating camera to save view vector on object
		//pose.orientation.z =  - cos(0.5*phi);
		//pose.orientation.w = + sin(0.5*phi);
		q = Quanterion3f_from_RPY(0,theta,phi);
		pose.orientation.w = q.getW();
		pose.orientation.x = q.getX();
		pose.orientation.y = q.getY();
		pose.orientation.z = q.getZ();

		px = (W*nx)/(r*v_ang*(PI/180));
		py = (H*ny)/(r*v_ang*(PI/180));
		x = ((nx-px)/2);
		y = ((ny-py)/2);

		modelstate.model_name ="flea3";
		modelstate.pose = pose;
		modelstate.twist = twist;
		setmodelstate.request.model_state=modelstate;
		client.call(setmodelstate);

		// updating view vector
		//theta = theta + 0.1*PI/180;
		//ROS_INFO(" phi = %f",sin(phi));
		ros::spinOnce();
	} 

	//modelstate_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	//ros::Timer markers_timer = n.createTimer(ros::Duration(1.0), PoseTimerCallBack);
	
   }


/**
 *   utility function for creating fcl Quanterion from Roll, Pitch and Yaw
 
Quaternion3f Quanterion3f_from_RPY(float Roll,float Pitch,float Yaw)
{
	tf::Matrix3x3 obs_mat;
	obs_mat.setEulerYPR(Yaw,Pitch,Roll);

	tf::Quaternion q_tf;
	obs_mat.getRotation(q_tf);

	Quaternion3f q(q_tf.getW(),q_tf.getX(),q_tf.getY(),q_tf.getZ());
	return(q);
}*/
