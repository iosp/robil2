// ** Written By : Sagi Vald **
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "robil_msgs/AssignNavTask.h"





float x,y;
robil_msgs::AssignNavTask wayps;
ros::Publisher marker_pub;

void pathCallBack(const robil_msgs::AssignNavTask& path)
{
  ROS_DEBUG(" Recieved Global Path ");
	 wayps = path;
}


void markersTimerCallBack (const ros::TimerEvent&)
{
    
    
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "WORLD";
    marker.header.stamp = ros::Time::now();
    int i;
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    for (i=0;i<wayps.waypoints.size();i++)
    {
	    marker.id = i;
		//ROS_INFO("Way No : %d",i);
	    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	    marker.type = visualization_msgs::Marker::SPHERE;

	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    marker.action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = wayps.waypoints[i].pose.pose.position.x;
	    marker.pose.position.y = wayps.waypoints[i].pose.pose.position.y;
	    marker.pose.position.z = 0;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 1.0;
	    marker.scale.y = 1.0;
	    marker.scale.z = 1.0;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 1.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 1.0;

	    marker.lifetime = ros::Duration(100);
		
	    // Publish the marker
	    while (marker_pub.getNumSubscribers() < 1)
	    {
	      if (ros::ok())
	      {
	      ROS_WARN_ONCE("Please create a subscriber to the marker");
	      sleep(1);
	      }
	    }
   ROS_DEBUG("id = %d" , marker.id  );
   ROS_DEBUG("Marker's X =  %f , Marker's Y =  %f",marker.pose.position.x,marker.pose.position.y);
    marker_pub.publish(marker);
     }

}




int main( int argc, char** argv )
{

  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("/OCU/SMME/NavigationTask", 10, pathCallBack);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  
  ros::Timer markers_timer = n.createTimer(ros::Duration(4.0), markersTimerCallBack);
  
  ros::spin();
}


