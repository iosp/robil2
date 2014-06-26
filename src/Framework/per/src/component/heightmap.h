#ifndef HEIGHTMAP__H
#define HEIGHTMAP__H
#include "helpermath.h"
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/**
    Robil-2 heightmap for the mapping component.
    Designed according to specs.
*/

#define TYPE_UNSCANNED 0
#define TYPE_CLEAR 1
#define TYPE_OBSTACLE 2
#define FEATURE_UNKNOWN	0
#define FEATURE_ROAD	1

class HeightMap
{
    public:
        HeightMap(int width, int height);
        ~HeightMap();
        
        /**
	 * These functions were planned to be implemented for packing & unpacking the map. But since
	 * we are working with the usual ROS messages for map we do not need these anymore.
	 */
	void    importFromArray(double* arr);
        void    exportToArray(double* arr);
        
	/**
	 * These are used to assign and query heights on the map given an absolute global position with
	 * coordinates transformed to cell coordinates.
	 * This is useful for working with the map from the outside world.
	 */
        void    setAbsoluteHeightAt(int x, int y, double height);
        double  getAbsoluteHeightAt(int x, int y);
	
	/**
	 * These are used to assign and query heights on the map given a relative position
	 * on the map itself. (0,0) would be the map center, regardless of what (0,0) is in the real world.
	 * This is useful for working with map data.
	 */
        void    setRelativeHeightAt(int x, int y, double height);
        double  getRelativeHeightAt(int x, int y);
	
	void    setRelativeTypeAt(int x, int y, int type);
        int     getRelativeTypeAt(int x, int y);
	
	void    setRelativeFeatureAt(int x, int y, int feature);
        int     getRelativeFeatureAt(int x, int y);
        
	/**
	 * This function attempts to (for now) classify each map cell as
	 * passable, obstacle or unknown. Expected to work at lightning speed.
	 */
        void 	calculateTypes();
	
	/**
	 * These functions receive the bobcat x and y coordinates (as real world coordinates)
	 * and it's rotation, and chop off a map according to IAI specs relative to the bobcat's front direction.
	 */
	HeightMap deriveMap(int px, int py, Rotation rot);
	HeightMap deriveMiniMap(int px, int py, Rotation rot);
	
	/**
	 * These functions are tools for a visual representation of the heightmap. 
	 * 3D is not yet implemented, and console representation is only partial.
	 * Bobcat position is in represented in absolute global coordinates.
	 */
        void displayConsole();
        void displayGUI(int, int, int, int enlarger=3);
        void displayTypesGUI(int enlarger=3);
        void display3D();
	
	/**
	 * These are getters for the internal data structures of the map. 
	 */
	vector<double>& getHeights();
        vector<int>& getTypes();
	vector<int>& getFeatures();
        
    private:
        
	/**
	 * These functions are used when the bobcat left far away from the combat area and we need to allocate 
	 * space for the new map data and forget some of the old map data (the one that is the farthest from us)
	 */
	void shiftLeft(); void shiftRight(); void shiftUp(); void shiftDown(); 
        
      
      
	double&                 _at(int x, int y);
	int&                 	 _typeAt(int x, int y);
	int&                 	 _featureAt(int x, int y);
        
        vector<double>          _heights;  
        vector<int>		 _types, _features;
        int                     _width;
        int                     _height;
        double                  _min, _max;
        Mat			 _compass;
        Mat			 _arrow;
	
	Vec2D			 _refPoint; //(0,0) in heightmap is refPoint(x*5,y*5) on the real world.
};
    
    
    



#endif
