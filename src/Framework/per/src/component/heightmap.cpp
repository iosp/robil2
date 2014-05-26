#include "heightmap.h"
#include "rdbg.h"
#include <cstdio>

using namespace cv;

#define HEIGHT_UNKNOWN -100.0

HeightMap::HeightMap(int width, int height)
{
    _heights.resize(width*height, HEIGHT_UNKNOWN);
    _types.resize(width*height, TYPE_UNSCANNED);
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

void HeightMap::setRelativeHeightAt(int x, int y, double height)
{
   
    if(x >= _width/2) { shiftRight(); return; }
    else if(x <= -_width/2) { shiftLeft(); return; }
    
    if(y >= _height/2) { shiftUp(); return; }
    else if(y <= -_height/2) {  shiftDown(); return; }
    
    x = _width/2 - x;
    y = _height/2 - y;
    
    if(_at(x,y) < _min) _at(x,y) = height;
    else _at(x,y) = _at(x,y)*0.8 + height*0.2;
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
      double x1 = _width/2 - x;
      double y1 = _height/2 - y;
      _at(x1,y1) = h;
    }
  rdbg("left");
}

void HeightMap::shiftRight()
{
  _refPoint = _refPoint.add(Vec2D(_width/2, 0));
  for(int y = -_height/2+1; y <= _height/2-1; y++)
    for(int x = -_width/2+1; x <= _width/2-1; x++)
    {
      double h = getRelativeHeightAt(x+_width/2, y);
      double x1 = _width/2 - x;
      double y1 = _height/2 - y;
      _at(x1,y1) = h;
    }
  rdbg("left");
  
}
void HeightMap::shiftUp()
{
  _refPoint = _refPoint.add(Vec2D(0, _height/2));
  for(int y = -_height/2+1; y <= _height/2-1; y++)
    for(int x = _width/2-1; x >= -_width/2+1; x--)
    {
      double h = getRelativeHeightAt(x, y+_height/2);
      double x1 = _width/2 - x;
      double y1 = _height/2 - y;
      _at(x1,y1) = h;
    }
  rdbg("up");
}

void HeightMap::shiftDown()
{
  _refPoint = _refPoint.add(Vec2D(0, -_height/2));
  for(int y = _height/2-1; y >= -_height/2+1; y--)
    for(int x = _width/2-1; x >= -_width/2+1; x--)
    {
      double h = getRelativeHeightAt(x, y-_height/2);
      double x1 = _width/2 - x;
      double y1 = _height/2 - y;
      _at(x1,y1) = h;
    }
  rdbg("down");
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

/*
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
*/

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
    
    // cout << "img " << _compass.rows << " " << _compass.cols << endl;
    /*
    for(int i = 0; i < _compass.rows; i++)
      for(int j = 0; j < _compass.cols; j++)
	if(_compass.at<Vec3b>(i,j) != Vec3b(255,255,255)) image.at<Vec3b>(i/2, j/2) = _compass.at<Vec3b>(i, j);
    */
    char name[30];
    sprintf(name, "GUI %d", _width);
    imshow(name, image);
    //printf("min: %g max: %g\n",_min,_max);
    //printf("error: %g\n", _min-_max);
// rdbg("gui exit");
 
    //cv::waitKey(1);
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
  cvtColor(image, image, CV_HSV2BGR);
  imshow("TerrainTypeUI", image);
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



