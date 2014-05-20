#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <robil_msgs/IEDLocation.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>

#include "ParameterHandler.h"

using namespace ros;
using namespace std;

void iedSim_callback(const robil_msgs::IEDLocation::ConstPtr & msg)
{
	cout << "got message" <<endl;
	cout << *msg <<endl;
	cout << "end_message" <<endl;
}

void printMenu()
{
	cout << "IEDSIM_TEST" <<endl;
	cout << "============" <<endl;
	cout << "1 - spawn/move ied gazebo coordinates" <<endl;
	cout << "2 - send a dummy location message - only works with the Sahar in the simulation" <<endl;
	cout << "h - show menu" <<endl;
	cout << "q - quit" <<endl;
	cout << "------------" <<endl;
	cout << "enter option:" <<endl;
}

int main(int argc,char**argv)
{
	ros::init(argc, argv,"IEDSIM_TEST");
	NodeHandle _nh;
	Publisher pub_CustomIED= _nh.advertise<robil_msgs::IEDLocation>(fetchParam(&_nh,"IEDSIM","CustomIED","sub"), 1);
	Publisher pub_Location = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(fetchParam(&_nh,"IEDSIM","Location","sub"),1);
	Subscriber sub_IEDLocation=_nh.subscribe(fetchParam(&_nh,"IEDSIM","IEDLocation","pub"),10,iedSim_callback);

	robil_msgs::IEDLocation msg1;
	geometry_msgs::PoseWithCovarianceStamped msg2;
	printMenu();

	char option =0;
	cin>> option;
	while(option!='q')
	{
		if(option=='1')
		{
			cout << "spawn/move ied - enter x y z values" <<endl;
			double x,y,z;
			cout << "x:" <<endl;
			cin>> x;
			cout << "y:" <<endl;
			cin>> y;
			cout << "z:" <<endl;
			cin>> z;
			msg1.is_detected=0;
			msg1.location.x=x;
			msg1.location.y=y;
			msg1.location.z=z;			
			pub_CustomIED.publish(msg1);
			spinOnce();
			cout << "message sent" <<endl;
		}
		else if(option=='2')
		{
			cout << "publishing :" <<endl;
			Rate r(1);
			for(int i=0;i<5;i++){			
				pub_Location.publish(msg2);
				spinOnce();
				r.sleep();
			}
			cout << "message sent 5 times" <<endl;
		}
		else if(option=='h')
		{
			printMenu();
		}
		else{
			cout << "invalid option" <<endl;
		}

		cout << "enter option:" <<endl;
		cin >> option;
	}
}
