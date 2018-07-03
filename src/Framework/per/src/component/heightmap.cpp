#include "heightmap.h"
#include "rdbg.h"
#include <cstdio>
#include <ros/ros.h>
#include <ros/package.h>


using namespace cv;

HeightMap::HeightMap(int width, int height, per::configConfig *p=NULL)
{
    _dynamic = p;
    _heights.resize(width*height, HEIGHT_UNKNOWN);
    _probabilities.resize(width*height, 0.0);
    _types.resize(width*height, TYPE_UNSCANNED);
    _features.resize(width*height, FEATURE_UNKNOWN);
    _width = width;
    _height = height;
    _min = -3;
    _max = 3;
#if ROS_VERSION_MINOR == 11 //Jade
    _compass = imread("compass.png");
    _arrow = imread("arrow.png");
#else //Kinetic or more
    std::string path = ros::package::getPath("per");
    path += "/img/";
    _compass = imread(path+"compass.png");
    _arrow = imread(path+"arrow.png");
#endif
    _refPoint = Vec2D(0,0);
}

HeightMap::~HeightMap()
{
    _heights.clear();
}


std::vector<int> getConvolution(string str, int size)
{
    std::vector<int> vect, conv;

    std::stringstream ss(str);

    int i;

    while (ss >> i)
    {
        vect.push_back(i);

        if (ss.peek() == ',')
            ss.ignore();
    }
    if (!size%2)
        size--;
    if (vect.size() != size)
        vect.assign(size, 1);
    conv.assign(size*size, 1);
    int mid = vect.size() / 2;
    for (int i = 0; i < vect.size(); i++)
    {
        for (int j = 0; j < vect.size(); j++)
        {
            if (i > mid)
            {
                if (i > j && j > vect.size() - i - 1)
                    conv.at(i * vect.size() + j) = vect.at(i);
                else
                    conv.at(i * vect.size() + j) = vect.at(j);
            }
            else if (i < mid)
            {
                if (i < j && j < vect.size() - i - 1)
                    conv.at(i * vect.size() + j) = vect.at(i);
                else
                    conv.at(i * vect.size() + j) = vect.at(j);
            }
            else
                conv.at(i * vect.size() + j) = vect.at(j);
        }

    }

    return conv;
}

double HeightMap::calc_height(int x, int y, std::vector<int> conv)
{
    if (_at(x, y) == HEIGHT_UNKNOWN)
        return HEIGHT_UNKNOWN;
    double height = 0;
    double counter = 0;
    int radius = (int)sqrt(conv.size());
    int k = 0;
    for (int i = x - radius / 2; i <= x + radius / 2; i++)
        for (int j = y - radius / 2; j <= y + radius / 2; j++)
        {
            double mul = conv.at(k++);
            if (i < 0 || i > _width || j < 0 || j > _height || _at(i, j) == HEIGHT_UNKNOWN)
                return 0.0;
            counter += abs(mul);
            height += mul * _at(i, j);
        }
//    if (_at(x, y) > 0.4)
//        cout << counter << ", " << height << " = " << height / counter << endl;
    if (counter)
      height /= (1.0 * counter);

    return height;
}

double HeightMap::calc_slope(int x, int y, std::vector<int> conv)
{
    if (_at(x, y) == HEIGHT_UNKNOWN)
        return HEIGHT_UNKNOWN;
    double height = 0;
    double counter = 0;
    int radius = (int)sqrt(conv.size());
    int k = 0;
    for (int i = x - radius; i <= x + radius; i++)
        for (int j = y - radius; j <= y + radius; j++)
        {
            double mul = conv.at(k++);
            if (i < 0 || i > _width || j < 0 || j > _height || _at(i, j) == HEIGHT_UNKNOWN)
                continue;
            counter += abs(mul);
            height += mul * _at(i, j);
        }
    height /= counter;
    return height;
}


void sobel_conv()
{

}

void HeightMap::calculateTypes()//Vec3D position, Rotation myRot)
{
    int mul;
    std::vector<int> conv = getConvolution(_dynamic->convolution_type, _dynamic->convolution_size);
    int conv_x[] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
    std::vector<int> slope_conv_x; slope_conv_x.assign(conv_x, conv_x+9);
    int conv_y[] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
    std::vector<int> slope_conv_y; slope_conv_y.assign(conv_y, conv_y+9);
    const int road_thresh = 5;
    for(int i = 1; i < _width-1; i++)
    {
        for(int j = 1; j < _height-1; j++)
        {
            double height = calc_height(i,j,conv);
            if (height > _dynamic->obstacle_threshold)
            {
                double gx = calc_height(i, j, slope_conv_x);
                double gy = calc_height(i, j, slope_conv_y);
                if (sqrt(gx * gx + gy * gy) > _dynamic->slope_threshold)
                  {
                    _types[j*_width+i] = TYPE_OBSTACLE;
//                    std::cout << i << ", " << j << ": " << gx << ", " << gy << std::endl;
                  }
                else
                    _types[j*_width+i] = TYPE_CLEAR;
            }
            else if(height == HEIGHT_UNKNOWN)
                _types[j*_width+i] = TYPE_UNSCANNED;
            else
                _types[j*_width+i] = TYPE_CLEAR;

        }
    }
    /**
      Perform smoothing
      **/
//    std::cout << "===================\n";
    for(int i = 1; i < _width-1; i++)
        for(int j = 1; j < _height-1; j++)
            if (_types[j*_width+i] == TYPE_OBSTACLE)
            {
                int counter = 0;
                for (int k = -_dynamic->reg_size; k < _dynamic->reg_size+1; k++)
                    for (int l = -_dynamic->reg_size; l < _dynamic->reg_size+1; l++)
                    {
                        if (i+k < 0 || i+k > _width || j+l < 0 || j+l > _height)
                            continue;
                        if (_types[(j+l)*_width+(k+i)] == TYPE_OBSTACLE)
                            counter++;
                    }
                if (counter < _dynamic->min_size)
                    _types[j*_width+i] = TYPE_CLEAR;
                else
                  {
                     _probabilities[j*_width+i] += 1;
                     if (_probabilities[j*_width+i] < _dynamic->repetitions)
                       _types[j*_width+i] = TYPE_CLEAR;
                  }

            }
//    std::cout << "===================\n";
}

HeightMap HeightMap::deriveMap(int px, int py, Rotation r)
{
    //according to ROBIL map specs
    px*=5;
    py*=5; //transformation from real-world coords [m] to 20x20cm cell coords
    px -= _refPoint.x;
    py -= _refPoint.y;
    HeightMap ans(150, 150);
    Quaternion q = GetFromRPY(r);
    Vec3D right3 = GetRightVector(q.x,q.y,q.z,q.w);
    Vec3D front3 = GetFrontVector(q.x,q.y,q.z,q.w);
    Vec2D front(front3.x, front3.y);
    Vec2D right(right3.x, right3.y);
    front = front.normalize();
    right = right.normalize();
    Vec2D pos(px,py);
    for(int i = -74; i < 74; i++)
        for(int j = -74; j < 74; j++)
        {
            Vec2D pt = pos.add(front.multiply(-j+25)).add(right.multiply(i));
            ans.setRelativeHeightAt(-j,i, getRelativeHeightAt(pt.x, pt.y));
            ans.setRelativeTypeAt(-j,i, getRelativeTypeAt(pt.x, pt.y));
            ans.setRelativeFeatureAt(-j,i, getRelativeFeatureAt(pt.x, pt.y));
        }
    return ans;
}

HeightMap HeightMap::deriveMiniMap(int px, int py, Rotation r)
{
    //according to ROBIL map specs
    px*=5;
    py*=5; //transformation from real-world coords [m] to 20x20cm cell coords
    px -= _refPoint.x;
    py -= _refPoint.y;
    HeightMap ans(50, 30);
    Quaternion q = GetFromRPY(r);
    Vec3D right3 = GetRightVector(q.x,q.y,q.z,q.w);
    Vec3D front3 = GetFrontVector(q.x,q.y,q.z,q.w);
    Vec2D front(front3.x, front3.y);
    Vec2D right(right3.x, right3.y);
    front = front.normalize();
    right = right.normalize();
    Vec2D pos(px,py);
    for(int i = -14; i < 14; i++)
        for(int j = -24; j < 24; j++)
        {
            Vec2D pt = pos.add(front.multiply(-j+25)).add(right.multiply(i));
            ans.setRelativeHeightAt(-j,i, getRelativeHeightAt(pt.x, pt.y));
            ans.setRelativeTypeAt(-j,i, getRelativeTypeAt(pt.x, pt.y));
            ans.setRelativeFeatureAt(-j,i, getRelativeFeatureAt(pt.x, pt.y));
        }
    return ans;
}



