#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PolygonStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <sensor_msgs/Image.h>
#include "std_msgs/Float64MultiArray.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include "displayImage/entropy.h"
#include "displayImage/laneFinder.h"
#include "displayImage/line23d.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */




using namespace cv;
using namespace std;
// using namespace per;

ros::NodeHandle *n;
ros::Publisher road_pub ;
bool lock = false;
int toDebug=0,toDebugLane=0;
int counter = 0;
int everyNthTime = 5;//should be : 2;

void displayImage(const sensor_msgs::ImageConstPtr& msg)
{
    if(lock)
    {
        cout << "locked" << endl;
        return;
    }
    Mat m = cv_bridge::toCvShare(msg, "bgr8")->image;//imdecode(Mat(msg.data),1);
    Mat gray, cropped;
    cvtColor(m, gray, CV_BGR2GRAY);
    Size s = gray.size();
    s.height = s.height/2;
    cv::Rect ROI(0, 964/2, 1288, 964/2);
    cropped = m(ROI);


    Mat ent_image;
    lock = true;

    clock_t t, t1, t2;
    t = clock();
    ros::param::param("/Road/Debug", toDebug,toDebug);
    ent_image = displayMyEntropy(cropped, 50, 100, toDebug);
    t1 = clock();
    vector<polydat> ps = find_lanes(ent_image,toDebugLane);
    t2 = clock();
    vector<geometry_msgs::Point32> points = extract_3d_from_lines(ps);
    //printf ("%.3f[s] = %.3f[s-entropy] + %.3f[s-lanes].\n", ((float)(t2-t))/CLOCKS_PER_SEC,((float)(t2-t1))/CLOCKS_PER_SEC,((float)(t1-t))/CLOCKS_PER_SEC);

    geometry_msgs::PolygonStamped poly;
    poly.header.frame_id = "road_poly";
    poly.header.stamp = ros::Time::now();
    for(vector<geometry_msgs::Point32>::iterator it=points.begin(); it != points.end();it++)
        poly.polygon.points.push_back(*it);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0, 0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    road_pub.publish(poly);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", poly.header.frame_id));
    lock = false;
}


void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{
    /**
      * chatterCallback:
     **/

    if(counter%everyNthTime == 0)
    {
        displayImage(msg);
        counter = 1;
    }
    else
    {
        counter ++;
    }
}




int main(int argc, char **argv)
{
    /**
    * main
   **/
    for(int i=1;i<argc;i++)
    {
        if(!strcmp(argv[i], "-d"))
            toDebug = 1;
        if(!strcmp(argv[i], "-l"))
            toDebugLane =1;
    }
    calculate_entropy_values();
    ros::init(argc, argv, "listener");
    ros::NodeHandle n1;
    n = &n1;
    ros::Subscriber sub = n->subscribe("SENSORS/CAM/R", 1000, chatterCallback);
    road_pub = n->advertise<geometry_msgs::PolygonStamped&>("/Road/Polygon",1000);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
