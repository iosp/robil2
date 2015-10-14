#ifndef LINE23D_H
#define LINE23D_H
#include <geometry_msgs/Point32.h>
#define FOV 1.3962634 //Field of view of camera
#define HEIGHT 1.97 //height of cameras in [m]
#define FX SIZE_Y / tan(FOV / 2) //focal length x
#define FY SIZE_X / 2 / tan(FOV / 2) //focal length y

geometry_msgs::Point32 extract_3d_from_pixel(cv::Point px, double fx=FX, double fy=FY)
{
    geometry_msgs::Point32 p;
    p.z = 0;
    p.x = fx * HEIGHT  / px.y;
    p.y = -p.x * (px.x - SIZE_X/2) / fy;
    return p;
}

vector<geometry_msgs::Point32> extract_3d_from_lines(vector<polydat> lines)
{
    vector<geometry_msgs::Point32> points;
    for (vector<polydat>::iterator it=lines.begin(); it != lines.end(); it++)
    {
        points.push_back(extract_3d_from_pixel((*it).p1));
        points.push_back(extract_3d_from_pixel((*it).p2));
    }
    return points;
}

#endif // LINE23D_H

