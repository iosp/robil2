#include "heightmap.h"
#include <cstdio>

using namespace cv;

HeightMap::HeightMap(int width, int height)
{
    _heights.resize(width*height, -100);
    _width = width;
    _height = height;
    _min = -2;
    _max = 6;
    _compass = imread("compass.png");
    _arrow = imread("arrow.png");
    //if(_compass.data == NULL) printf("No data!\n");
}

HeightMap::~HeightMap()
{
    _heights.clear();
}

double& HeightMap::_at(int x, int y)
{
    //printf("%d\n", y*_width+x);
    return _heights[y*_width+x];
}

void HeightMap::setHeightAt(int x, int y, double height)
{
    
    if(height < _min) _min = height;
    if(height > _max) _max = height;
   
    if(x >= _width/2 || x <= -_width/2) return;
    if(y >= _height/2 || y <= -_height/2) return;
    
    x = _width/2 - x;
    y = _height/2 - y;
    
    if(_at(x,y) == -100) _at(x,y) = height;
    else _at(x,y) = _at(x,y)*0.8 + height*0.2;
}

void HeightMap::displayConsole()
{
    for(int y = 0; y < _height; y++)
    {
        for(int x = 0; x < _width; x++)
        {
            printf("%3.3f\t", this->_at(x,y));
        }
        printf("\n");
    }
}

void HeightMap::displayGUI(int rotation, int px, int py)
{
    int enlarger = 1;
    Mat image(_width*enlarger, _height*enlarger, CV_8UC3);
    cvtColor(image, image, CV_BGR2HSV);
    
    for(int y = 0; y < _height*enlarger; y++)
        for(int x = 0; x < _width*enlarger; x++)
        {
            double h = this->_at(x/enlarger,y/enlarger);
            if(h < -10) 
            {
                image.at<Vec3b>(x,y) = Vec3b(0,0,200);
                continue;
            }
            double c = (h - _min)/(_max-_min);
            image.at<Vec3b>(x,y) = Vec3b(120*(1-c),240, MIN(240*(c+0.2), 240));
        }
    cvtColor(image, image, CV_HSV2BGR);
  
   
  /*
  //put the tempory arrow representing my position and rotation on the map
    Mat arrow;
    cv::Point2f pt(_arrow.rows/2, _arrow.cols/2);
    cv::Mat r = cv::getRotationMatrix2D(pt, 90+rotation, 1.0);
    cv::warpAffine(_arrow, arrow, r, cv::Size(_arrow.rows, _arrow.rows));
    if(px >= _width/2 || px <= -_width/2) return;
    if(py >= _height/2 || py <= -_height/2) return;
    px = _width/2 - px;
    py = _height/2 - py;
    for(int i = 0; i < _arrow.rows; i++)
      for(int j = 0; j < _arrow.cols; j++)
	if(arrow.at<Vec3b>(i,j) != Vec3b(0,0,0)) image.at<Vec3b>(i+px-arrow.rows/2, j+py-arrow.cols/2) = arrow.at<Vec3b>(i, j);
    
 for(int i = 0; i < _compass.rows; i++)
      for(int j = 0; j < _compass.cols; j++)
	if(_compass.at<Vec3b>(i,j) != Vec3b(255,255,255)) image.at<Vec3b>(i, j) = _compass.at<Vec3b>(i, j);
    */
	
    imshow("oded", image);
    printf("min: %g max: %g\n",_min,_max);
    //printf("error: %g\n", _min-_max);
 
 
    cv::waitKey(1);
}

