#include "heightmap.h"
#include "rdbg.h"
#include <cstdio>

using namespace cv;

#define HEIGHT_UNKNOWN -100.0

HeightMap::HeightMap(int width, int height)
{
    _heights.resize(width*height, HEIGHT_UNKNOWN);
    _types.resize(width*height, TYPE_UNSCANNED);
    _features.resize(width*height, FEATURE_UNKNOWN);
    _width = width;
    _height = height;
    _min = -5;
    _max = 5;
    _compass = imread("compass.png");
    _arrow = imread("arrow.png");
    _refPoint = Vec2D(0,0);
   
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

int& HeightMap::_typeAt(int x, int y)
{
    return _types[y*_width+x];
}

int& HeightMap::_featureAt(int x, int y)
{
    return _features[y*_width+x];
}
void HeightMap::setRelativeHeightAt(int x, int y, double height)
{
   
    if(x >= _width/2) { shiftRight(); return; }
    else if(x <= -_width/2) { shiftLeft(); return; }
    
    if(y >= _height/2) { shiftUp(); return; }
    else if(y <= -_height/2) {  shiftDown(); return; }
    
    x = _width/2 - x;
    y = _height/2 - y;
    
    if(_at(x,y) < _min) _at(x,y) = height;
    else _at(x,y) = _at(x,y)*0.95 + height*0.05;
}

void HeightMap::setRelativeTypeAt(int x, int y, int type)
{
   
    if(x >= _width/2) { shiftRight(); return; }
    else if(x <= -_width/2) { shiftLeft(); return; }
    
    if(y >= _height/2) { shiftUp(); return; }
    else if(y <= -_height/2) {  shiftDown(); return; }
    
    x = _width/2 - x;
    y = _height/2 - y;
    
    _typeAt(x,y) = type;
}

void HeightMap::setRelativeFeatureAt(int x, int y, int feature)
{
   
    if(x >= _width/2) { shiftRight(); return; }
    else if(x <= -_width/2) { shiftLeft(); return; }
    
    if(y >= _height/2) { shiftUp(); return; }
    else if(y <= -_height/2) {  shiftDown(); return; }
    
    x = _width/2 - x;
    y = _height/2 - y;
    
    _featureAt(x,y) = feature;
}

void HeightMap::setAbsoluteHeightAt(int x, int y, double height)
{
    x -= _refPoint.x;
    y -= _refPoint.y;
    if(height < HEIGHT_UNKNOWN+5) return;
    if(height < _min) _min = height;
    if(height > _max) _max = height;
    setRelativeHeightAt(x,y,height);
}

void HeightMap::shiftLeft()
{
  _refPoint = _refPoint.add(Vec2D(-_width/2, 0));
  for(int y = -_height/2+1; y <= _height/2-1; y++)
    for(int x = _width/2-1; x >= -_width/2+1; x--)
    {
      double h = getRelativeHeightAt(x-_width/2, y);
      int t = getRelativeTypeAt(x-_width/2, y);
      int f = getRelativeFeatureAt(x-_width/2, y);
      double x1 = _width/2 - x;
      double y1 = _height/2 - y;
      _at(x1,y1) = h;
      _featureAt(x1,y1) = f;
      _typeAt(x1,y1) = t;
    }
  //rdbg("left");
}

void HeightMap::shiftRight()
{
  _refPoint = _refPoint.add(Vec2D(_width/2, 0));
  for(int y = -_height/2+1; y <= _height/2-1; y++)
    for(int x = -_width/2+1; x <= _width/2-1; x++)
    {
      double h = getRelativeHeightAt(x+_width/2, y);
      int t = getRelativeTypeAt(x+_width/2, y);
      int f = getRelativeFeatureAt(x+_width/2, y);
      double x1 = _width/2 - x;
      double y1 = _height/2 - y;
      _at(x1,y1) = h;
      _featureAt(x1,y1) = f;
      _typeAt(x1,y1) = t;
    }
  //rdbg("left");
  
}
void HeightMap::shiftUp()
{
  _refPoint = _refPoint.add(Vec2D(0, _height/2));
  for(int y = -_height/2+1; y <= _height/2-1; y++)
    for(int x = _width/2-1; x >= -_width/2+1; x--)
    {
      double h = getRelativeHeightAt(x, y+_height/2);
      int t = getRelativeTypeAt(x, y+_height/2);
      int f = getRelativeFeatureAt(x, y+_height/2);
      double x1 = _width/2 - x;
      double y1 = _height/2 - y;
      _at(x1,y1) = h;
      _featureAt(x1,y1) = f;
      _typeAt(x1,y1) = t;
    }
  //rdbg("up");
}

void HeightMap::shiftDown()
{
  _refPoint = _refPoint.add(Vec2D(0, -_height/2));
  for(int y = _height/2-1; y >= -_height/2+1; y--)
    for(int x = _width/2-1; x >= -_width/2+1; x--)
    {
      double h = getRelativeHeightAt(x, y-_height/2);
      int t = getRelativeTypeAt(x, y-_height/2);
      int f = getRelativeFeatureAt(x, y-_height/2);
      double x1 = _width/2 - x;
      double y1 = _height/2 - y;
      _at(x1,y1) = h;
      _featureAt(x1,y1) = f;
      _typeAt(x1,y1) = t;
    }
  //rdbg("down");
}

double HeightMap::getRelativeHeightAt(int x, int y)
{
  if(x >= _width/2 || x <= -_width/2) return HEIGHT_UNKNOWN;
  if(y >= _height/2 || y <= -_height/2) return HEIGHT_UNKNOWN;
  x = _width/2 - x;
  y = _height/2 - y;
  return _at(x,y);
}

double HeightMap::getAbsoluteHeightAt(int x, int y)
{
  x -= _refPoint.x;
  y -= _refPoint.y;
  return getRelativeHeightAt(x,y);
}

int HeightMap::getRelativeTypeAt(int x, int y)
{
  if(x >= _width/2 || x <= -_width/2) return TYPE_UNSCANNED;
  if(y >= _height/2 || y <= -_height/2) return TYPE_UNSCANNED;
  x = _width/2 - x;
  y = _height/2 - y;
  return _typeAt(x,y);
}

int HeightMap::getRelativeFeatureAt(int x, int y)
{
  if(x >= _width/2 || x <= -_width/2) return FEATURE_UNKNOWN;
  if(y >= _height/2 || y <= -_height/2) return FEATURE_UNKNOWN;
  x = _width/2 - x;
  y = _height/2 - y;
  return _featureAt(x,y);
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

void HeightMap::displayGUI(int rotation, int px, int py, int enlarger)
{
   // rdbg("gui enter");
    Mat image(_width*enlarger, _height*enlarger, CV_8UC3);
    cvtColor(image, image, CV_BGR2HSV);
    
    for(int y = 0; y < _height*enlarger; y++)
        for(int x = 0; x < _width*enlarger; x++)
        {
            double h = this->_at(x/enlarger,y/enlarger);
            if(h <= _min) 
            {
                image.at<Vec3b>(x,y) = Vec3b(0,0,100);
                continue;
            }
            double c = (h - _min)/(_max-_min);
            image.at<Vec3b>(x,y) = Vec3b(120*(1-c),240, 240);
        }
    cvtColor(image, image, CV_HSV2BGR);
    //put the tempory arrow representing my position and rotation on the map
    Mat arrow;
    cv::Point2f pt(_arrow.rows/2, _arrow.cols/2);
    cv::Mat r = cv::getRotationMatrix2D(pt, 90+rotation, 1.0);
    cv::warpAffine(_arrow, arrow, r, cv::Size(_arrow.rows, _arrow.rows));
    px *= 5;
    py *= 5; //transformation to 20x20cm cell coords
    px -= _refPoint.x;
    py -= _refPoint.y;
    if(px >= _width/2 || px <= -_width/2 || py >= _height/2 || py <= -_height/2) 
    {
      char name[30];
      sprintf(name, "%f %d", _refPoint.x, py);
      imshow(name, image);
      //cv::waitKey(1);
      return;
    }
    px = _width/2 - px;
    py = _height/2 - py;
    for(int i = 0; i < _arrow.rows; i++)
      for(int j = 0; j < _arrow.cols; j++)
	if(arrow.at<Vec3b>(i,j) != Vec3b(0,0,0)) image.at<Vec3b>(i+px*enlarger-arrow.rows/2, j+py*enlarger-arrow.cols/2) = arrow.at<Vec3b>(i, j);
    
  
    char name[30];
    sprintf(name, "GUI %d", _width);
    imshow(name, image);
   
}

void HeightMap::displayTypesGUI(int enlarger)
{
  //int enlarger = 3;
  Mat image(_width*enlarger, _height*enlarger, CV_8UC3);
  //cvtColor(image, image, CV_BGR2HSV);
  
  for(int y = 0; y < _height*enlarger; y++)
      for(int x = 0; x < _width*enlarger; x++)
      {
	  int t = this->_types[x/enlarger + _width*(y/enlarger)];
	  
	  if(t == TYPE_CLEAR) image.at<Vec3b>(x,y) = Vec3b(255,255,255);
	  else if(t == TYPE_OBSTACLE) image.at<Vec3b>(x,y) = Vec3b(255,0,0);
	  else if(t == TYPE_UNSCANNED) image.at<Vec3b>(x,y) = Vec3b(60,160,60);
	  int f = this->_features[x/enlarger + _width*(y/enlarger)];
	  if(f == FEATURE_ROAD) image.at<Vec3b>(x,y) = Vec3b(100,100,100);
      }
  //cvtColor(image, image, CV_HSV2BGR);
  int px = _width/2+25;
  int py = _height/2;
  Mat arrow;
  cv::Point2f pt(_arrow.rows/2, _arrow.cols/2);
  cv::Mat r = cv::getRotationMatrix2D(pt, 90, 1.0);
  cv::warpAffine(_arrow, arrow, r, cv::Size(_arrow.rows, _arrow.rows));
  for(int i = 0; i < _arrow.rows; i++)
      for(int j = 0; j < _arrow.cols; j++)
	if(arrow.at<Vec3b>(i,j) != Vec3b(0,0,0)) image.at<Vec3b>(i+px*enlarger-arrow.rows/2, j+py*enlarger-arrow.cols/2) = arrow.at<Vec3b>(i, j);
  imshow("TerrainTypeUI", image);
  cv::waitKey(1);
}

void HeightMap::calculateTypes()
{
  const double obs_thresh = 0.2;
  const int road_thresh = 5;
  for(int i = 1; i < _width-1; i++)
    for(int j = 1; j < _height-1; j++)
    {
      //if(_types[j*_width+i] != TYPE_UNSCANNED) continue;
      double height = _at(i, j);
      double heightx1 = _at(i-1, j);
      double heightx2 = _at(i+1, j);
      double heighty1 = _at(i, j-1);
      double heighty2 = _at(i, j+1);
      if(height == HEIGHT_UNKNOWN)
      {
	if(heightx1 != HEIGHT_UNKNOWN && heightx2 != HEIGHT_UNKNOWN) height = _at(i,j) = (heightx1 + heightx2)/2;
	else if(heighty1 != HEIGHT_UNKNOWN && heighty2 != HEIGHT_UNKNOWN) height = _at(i,j) = (heighty1 + heighty2)/2;
	else continue;
      }
      
      
      
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
      else _types[j*_width+i] = TYPE_UNSCANNED;
      //_types[j*_width+i] = TYPE_CLEAR;
      
      if(	i - road_thresh >= 0 && 
		j - road_thresh >= 0 &&
		i + road_thresh < _width-1 && 
		j + road_thresh < _height-1)
      {
	  bool isClear = true;
	  for(int x = i - road_thresh; x < i + road_thresh; x++)
	    for(int y = j - road_thresh; y < j + road_thresh; y++)
	    {
		if(_types[y*_width+x] != TYPE_CLEAR) isClear = false;
	    }
	  if(isClear) _features[(j*_width+i)] = FEATURE_ROAD;
	  else _features[(j*_width+i)] = FEATURE_UNKNOWN;
      }
      
    }
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


vector<double>& HeightMap::getHeights()
{
  return _heights;
}


vector<int>& HeightMap::getTypes()
{
  return _types;
}


vector<int>& HeightMap::getFeatures()
{
  return _features;
}






