#ifndef HEIGHTMAP_OBJECT_H
#define HEIGHTMAP_OBJECT_H
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
/**
    Robil-2 heightmap for the mapping component.
    Designed according to specs.
*/
#define OBSTACLE_THRESH 0.4
#define HEIGHT_UNKNOWN -100.0

#define TYPE_UNSCANNED 		0
#define TYPE_CLEAR 		1
#define TYPE_OBSTACLE 		2
#define FEATURE_UNKNOWN		0

#define FEATURE_ROAD		1
#define FEATURE_NOT_ROAD 	5

#define LANES_IMAGE_HEIGHT 964
#define LANES_IMAGE_WIDTH 1288


class HeightmapObject{
public:
    HeightmapObject(){}
    ~HeightmapObject(){}
    /**
     * These are used to assign and query heights on the map given an absolute global position with
     * coordinates transformed to cell coordinates.
     * This is useful for working with the map from the outside world.
     */
    void    setAbsoluteHeightAt(int x, int y, double height)
    {
        x -= _refPoint.x;
        y -= _refPoint.y;
        if(height < HEIGHT_UNKNOWN+5) return;
        //if(height < _min) _min = height;
        //if(height > _max) _max = height;
        setRelativeHeightAt(x,y,height);
    }
    double  getAbsoluteHeightAt(int x, int y)
    {
        x -= _refPoint.x;
        y -= _refPoint.y;
        return getRelativeHeightAt(x,y);
    }


    /// Walrus Experiment:
    void    setAbsoluteFeatureAt(int x, int y, int feature)
    {
        x -= _refPoint.x;
        y -= _refPoint.y;
        //if(height < HEIGHT_UNKNOWN+5) return;
        //if(height < _min) _min = height;
        //if(height > _max) _max = height;
        setRelativeFeatureAt(x,y,feature);
    }
    int getAbsoluteFeatureAt(int x, int y)
    {
        x -= _refPoint.x;
        y -= _refPoint.y;
        return getRelativeFeatureAt(x,y);
    }

    int getAbsoluteTypeAt(int x, int y)
    {

        x -= _refPoint.x;
        y -= _refPoint.y;
        //if(height < HEIGHT_UNKNOWN+5) return;
        //if(height < _min) _min = height;
        //if(height > _max) _max = height;
        getRelativeTypeAt(x,y);
    }
    /// Until here **

    /**
     * These are used to assign and query heights on the map given a relative position
     * on the map itself. (0,0) would be the map center, regardless of what (0,0) is in the real world.
     * This is useful for working with map data.
     */
    void setRelativeHeightAt(int x, int y, double height)
    {

        if(x >= _width/2) { shiftRight(); return; }
        else if(x <= -_width/2) { shiftLeft(); return; }

        if(y >= _height/2) { shiftUp(); return; }
        else if(y <= -_height/2) {  shiftDown(); return; }

        x = _width/2 - x;
        y = _height/2 - y;

        if(_at(x,y) < _min) _at(x,y) = height;
        else _at(x,y) = _at(x,y)*0.9 + height*0.1;
    }
    double  getRelativeHeightAt(int x, int y)
    {
        if(x >= _width/2 || x <= -_width/2) return HEIGHT_UNKNOWN;
        if(y >= _height/2 || y <= -_height/2) return HEIGHT_UNKNOWN;
        x = _width/2 - x;
        y = _height/2 - y;
        return _at(x,y);
    }

    void setRelativeTypeAt(int x, int y, int type)
    {

        if(x >= _width/2) { shiftRight(); return; }
        else if(x <= -_width/2) { shiftLeft(); return; }

        if(y >= _height/2) { shiftUp(); return; }
        else if(y <= -_height/2) {  shiftDown(); return; }

        x = _width/2 - x;
        y = _height/2 - y;

        _typeAt(x,y) = type;
    }
    int getRelativeTypeAt(int x, int y)
    {
        if(x >= _width/2 || x <= -_width/2) return TYPE_UNSCANNED;
        if(y >= _height/2 || y <= -_height/2) return TYPE_UNSCANNED;
        x = _width/2 - x;
        y = _height/2 - y;
        return _typeAt(x,y);
    }

    void setRelativeFeatureAt(int x, int y, int feature)
    {

        if(x >= _width/2) { shiftRight(); return; }
        else if(x <= -_width/2) { shiftLeft(); return; }

        if(y >= _height/2) { shiftUp(); return; }
        else if(y <= -_height/2) {  shiftDown(); return; }

        x = _width/2 - x;
        y = _height/2 - y;

        _featureAt(x,y) = feature;
    }
    int getRelativeFeatureAt(int x, int y)
    {
        if(x >= _width/2 || x <= -_width/2) return FEATURE_UNKNOWN;
        if(y >= _height/2 || y <= -_height/2) return FEATURE_UNKNOWN;
        x = _width/2 - x;
        y = _height/2 - y;
        return _featureAt(x,y);
    }

    vector<double>& getHeights(){return _heights;}
    vector<int>& getTypes(){return _types;}
    vector<int>& getFeatures(){return _features;}

    /**
     * These functions receive the bobcat x and y coordinates (as real world coordinates)
     * and it's rotation, and chop off a map according to IAI specs relative to the bobcat's front direction.
     */
    Mat add_arrow(Mat image, int rotation, int px, int py, int enlarger)
    {
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
          return image;
      }
      px = _width/2 - px;
      py = _height/2 - py;
      for(int i = 0; i < _arrow.rows; i++)
          for(int j = 0; j < _arrow.cols; j++)
              if(arrow.at<Vec3b>(i,j) != Vec3b(0,0,0))
                  image.at<Vec3b>(i+px*enlarger-arrow.rows/2, j+py*enlarger-arrow.cols/2) = arrow.at<Vec3b>(i, j);
      return image;
    }
    /**
     * These functions are tools for a visual representation of the heightmap.
     * 3D is not yet implemented, and console representation is only partial.
     * Bobcat position is in represented in absolute global coordinates.
     */
    void displayGUI(int rotation, int px, int py, int enlarger=3)
    {
        Mat image = this->generateMat(rotation, px, py, enlarger);
        //put the tempory arrow representing my position and rotation on the map
        image = add_arrow(image, rotation, px, py, enlarger);
        char name[30];
        sprintf(name, "GUI %d", _width);

        if (!image.empty())
        {
            imshow(name, image);
            waitKey(100);
        }

    }

    Mat generateMat(int rotation, int px, int py, int enlarger, bool arrow=false)
    {
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
        if (arrow)
            image = this->add_arrow(image, rotation, px, py, enlarger);
        return image;
    }

    Mat generateMat(int enlarger)
    {
        Mat image(_width*enlarger, _height*enlarger, CV_8UC3);
        //cvtColor(image, image, CV_BGR2HSV);

        for(int y = 0; y < _height*enlarger; y++)
        {
            for(int x = 0; x < _width*enlarger; x++)
            {
                int t = this->_types[x/enlarger + _width*(y/enlarger)];

                if(t == TYPE_CLEAR) image.at<Vec3b>(x,y) = Vec3b(255,255,255);
                else if(t == TYPE_OBSTACLE) image.at<Vec3b>(x,y) = Vec3b(255,0,0);
                else if(t == TYPE_UNSCANNED) image.at<Vec3b>(x,y) = Vec3b(60,160,60);

                int f = this->_features[x/enlarger + _width*(y/enlarger)];

                if(f == FEATURE_ROAD)
                {
                    image.at<Vec3b>(x,y) = Vec3b(100,100,100);
                }
                if(f == FEATURE_NOT_ROAD)
                {
                    image.at<Vec3b>(x,y) = Vec3b(60,100,150);
                }
            }
        }
        int px = _width/2+25;
        int py = _height/2;
        Mat arrow;
        cv::Point2f pt(_arrow.rows/2, _arrow.cols/2);
        cv::Mat r = cv::getRotationMatrix2D(pt, 90, 1.0);
        cv::warpAffine(_arrow, arrow, r, cv::Size(_arrow.rows, _arrow.rows));
        for(int i = 0; i < _arrow.rows; i++)
        {
            for(int j = 0; j < _arrow.cols; j++)
            {
                if(arrow.at<Vec3b>(i,j) != Vec3b(0,0,0))
                {
                    image.at<Vec3b>(i+px*enlarger-arrow.rows/2, j+py*enlarger-arrow.cols/2) = arrow.at<Vec3b>(i, j);
                }
            }
        }
        return image;
    }

    void displayTypesGUI(Mat lanes,int enlarger=3)
    {
        //int enlarger = 3;
        Mat image = generateMat(enlarger);
        if (image.empty())
            return;
        imshow("TerrainTypeUI", image);

        cv::waitKey(200);
    }

protected:
    void shiftLeft()
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
    void shiftRight()
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
    void shiftUp()
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
    void shiftDown()
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



    double& _at(int x, int y)
    {
        return _heights[y*_width+x];
    }
    int& _typeAt(int x, int y)
    {
        return _types[y*_width+x];
    }
    int&  _featureAt(int x, int y)
    {
        return _features[y*_width+x];
    }

    vector<double>          _heights, _probabilities;
    vector<int>		 _types, _features;
    int                     _width;
    int                     _height;
    double                  _min, _max;
    Vec2D			 _refPoint; //(0,0) in heightmap is refPoint(x*5,y*5) on the real world.
    Mat			 _compass;
    Mat			 _arrow;

};

#endif // HEIGHTMAP_OBJECT_H
