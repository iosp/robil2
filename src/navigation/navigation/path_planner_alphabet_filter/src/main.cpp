/*
 * main.cpp
 *
 *  Created on: Oct 7, 2015
 *      Author: dan
 */

#include "PathPlanAlphaBetaFilter.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;


typedef PathPlanAlphaBetaFilter::Point Point;
typedef PathPlanAlphaBetaFilter::Path Path;

cv::Mat img;
Point current_position;

Path p1, p2, rp;

PathPlanAlphaBetaFilter filter;



void print_path(const Path& path, cv::Scalar color, int r = 3)
{
	for(size_t i=1;i<path.size();i++)
	{
		cv::line(img, cv::Point(path[i-1].x, path[i-1].y),cv::Point(path[i].x, path[i].y), color);
	}
	for(size_t i=0;i<path.size();i++)
	{
		cv::circle(img, cv::Point(path[i].x, path[i].y), r, color, -1);
	}
	cv::imshow("win", img);
}

void refresh_img()
{
	img = cv::Mat( 1400, 1400, CV_8UC3, cvScalar(255,255,255) );
	current_position = Point( img.cols/2, img.rows/2 );
	cv::circle(img, cv::Point(current_position.x, current_position.y), 3, cvScalar(0,0,0), -1);

	print_path(p1, cvScalar(255, 0, 0));
	print_path(p2, cvScalar(0, 255, 0));
	print_path(rp, cvScalar(0, 0, 255));

}

void applay_filter()
{
    rp = filter.update(current_position, p1, p2, 0.2);
}

void on_mouse(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          Point p(x,y);
          p1.push_back( p );

          applay_filter();
          refresh_img();
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          Point p(x,y);
          p2.push_back( p );

          applay_filter();
          refresh_img();
     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
    	 p1.clear();
    	 p2.clear();
    	 rp.clear();
    	 refresh_img();
     }

}

int main(int a, char** aa)
{


	refresh_img();

	cv::namedWindow("win", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("win", on_mouse, NULL);


	cv::imshow("win", img);


	cv::waitKey();

	return 0;
}

