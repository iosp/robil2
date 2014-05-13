#include "heightmap.h"

#include <cstdio>

using namespace cv;

#define HEIGHT_UNKNOWN -100

HeightMap::HeightMap(int width, int height)
{
    _heights.resize(width*height, HEIGHT_UNKNOWN);
    _types.resize(width*height, TYPE_UNSCANNED);
    _width = width;
    _height = height;
    _min = -2;
    _max = 0;
    _compass = imread("compass.png");
    _arrow = imread("arrow.png");
    _mayGrow = false;
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
    if(height <= HEIGHT_UNKNOWN) return;
    if(height < _min) _min = height;
    if(height > _max) _max = height;
   
    if(x >= _width/2 || x <= -_width/2) { grow(); return; }
    if(y >= _height/2 || y <= -_height/2) { grow(); return; }
    
    x = _width/2 - x;
    y = _height/2 - y;
    
    if(_at(x,y) == HEIGHT_UNKNOWN) _at(x,y) = height;
    else _at(x,y) = _at(x,y)*0.8 + height*0.2;
}

double HeightMap::getHeightAt(int x, int y)
{
  if(x >= _width/2 || x <= -_width/2) return HEIGHT_UNKNOWN;
  if(y >= _height/2 || y <= -_height/2) return HEIGHT_UNKNOWN;
  x = _width/2 - x;
  y = _height/2 - y;
  return _at(x,y);
}

void HeightMap::grow()
{
  if(!_mayGrow) return;
  //rdbg("grow1");
  vector<double> heights;
  vector<int> types;
  int new_width = _width * 1.25;
  int new_height = _height * 1.25;
  heights.resize(new_width*new_height, HEIGHT_UNKNOWN);
  //rdbg("grow2");
  types.resize(new_width*new_height, TYPE_UNSCANNED);
  //rdbg("grow3");
  for(int i = -_width/2; i < _width/2; i++)
    for(int j = -_height/2; j < _height/2; j++)
    {
      int x = new_width/2 - i;
      int y = new_height/2 - j;
      //printf("%d/%d ", y*new_width+x, new_width*new_height);
      heights[y*new_width+x] = getHeightAt(i,j);
    }
  //rdbg("grow4");
  _width = new_width;
  _height = new_height;
  _heights = heights;
  _types = types;
  //rdbg("grow5");
  //calculateTypes();
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
    //put the tempory arrow representing my position and rotation on the map
    Mat arrow;
    cv::Point2f pt(_arrow.rows/2, _arrow.cols/2);
    cv::Mat r = cv::getRotationMatrix2D(pt, 90+rotation, 1.0);
    cv::warpAffine(_arrow, arrow, r, cv::Size(_arrow.rows, _arrow.rows));
    px *= 5;
    py *= 5;
    if(px >= _width/2 || px <= -_width/2 || py >= _height/2 || py <= -_height/2) 
    {
      imshow("oded", image);
      cv::waitKey(1);
      return;
    }
    px = _width/2 - px;
    py = _height/2 - py;
    for(int i = 0; i < _arrow.rows; i++)
      for(int j = 0; j < _arrow.cols; j++)
	if(arrow.at<Vec3b>(i,j) != Vec3b(0,0,0)) image.at<Vec3b>(i+px*enlarger-arrow.rows/2, j+py*enlarger-arrow.cols/2) = arrow.at<Vec3b>(i, j);
    
    // cout << "img " << _compass.rows << " " << _compass.cols << endl;
    for(int i = 0; i < _compass.rows; i++)
	  for(int j = 0; j < _compass.cols; j++)
	    if(_compass.at<Vec3b>(i,j) != Vec3b(255,255,255)) image.at<Vec3b>(i, j) = _compass.at<Vec3b>(i, j);
	
    char name[20];
    sprintf(name, "oded%d", _width);
    imshow(name, image);
    //printf("min: %g max: %g\n",_min,_max);
    //printf("error: %g\n", _min-_max);
 
 
    cv::waitKey(1);
}

void HeightMap::displayTypesGUI()
{
  int enlarger = 3;
  Mat image(_width*enlarger, _height*enlarger, CV_8UC3);
  //cvtColor(image, image, CV_BGR2HSV);
  
  for(int y = 0; y < _height*enlarger; y++)
      for(int x = 0; x < _width*enlarger; x++)
      {
	  int t = this->_types[x/enlarger + _width*(y/enlarger)];
	  
	  if(t == TYPE_CLEAR) image.at<Vec3b>(x,y) = Vec3b(255,255,255);
	  else if(t == TYPE_OBSTACLE) image.at<Vec3b>(x,y) = Vec3b(255,0,0);
	  else if(t == TYPE_UNSCANNED) image.at<Vec3b>(x,y) = Vec3b(160,160,255);
      }
  //cvtColor(image, image, CV_HSV2BGR);
  //imshow("oded2", image);
  cv::waitKey(1);
}

void HeightMap::calculateTypes()
{
  const double obs_thresh = 0.1;
  
  for(int i = 1; i < _width-1; i++)
    for(int j = 1; j < _height-1; j++)
    {
      double height = _at(i, j);
      if(height == HEIGHT_UNKNOWN) continue;
      double heightx1 = _at(i-1, j);
      double heightx2 = _at(i+1, j);
      double heighty1 = _at(i, j-1);
      double heighty2 = _at(i, j+1);
      
      
      if(heighty2 != HEIGHT_UNKNOWN && heighty1 != HEIGHT_UNKNOWN)
      {
	if(abs(heighty2-heighty1)/2 > obs_thresh) _types[j*_width+i] = TYPE_OBSTACLE;
	else _types[j*_width+i] = TYPE_CLEAR;
      }
      else if(heightx2 != HEIGHT_UNKNOWN && heightx1 != HEIGHT_UNKNOWN)
      {
	if(abs(heightx2-heightx1)/2 > obs_thresh) _types[j*_width+i] = TYPE_OBSTACLE;
	else _types[j*_width+i] = TYPE_CLEAR;
      }
      else _types[(j+1)*_width+i] = TYPE_UNSCANNED;
      //_types[j*_width+i] = TYPE_CLEAR;
	
    }
}

HeightMap HeightMap::getRelativeMap(int px, int py, Rotation r)
{
  //according to ROBIL map specs
  px*=5;
  py*=5;
  HeightMap ans(200, 200);
  Quaternion q = GetFromRPY(r);
  Vec3D right3 = GetRightVector(q.x,q.y,q.z,q.w);
  Vec3D front3 = GetFrontVector(q.x,q.y,q.z,q.w);
  Vec2D front(front3.x, front3.y);
  Vec2D right(right3.x, right3.y);
  front = front.normalize();
  right = right.normalize();
  Vec2D pos(px,py);
  for(int i = -99; i < 99; i++)
    for(int j = -99; j < 99; j++)
    {
      Vec2D pt = pos.add(front.multiply(-j)).add(right.multiply(i));
      ans.setHeightAt(-j,i, getHeightAt(pt.x, pt.y)); 
    }
  return ans;
}


vector<double>& HeightMap::getHeights()
{
  return _heights;
}


vector<int>& HeightMap::getTypes()
{
  return _types;
}



