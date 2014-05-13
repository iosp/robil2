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

class HeightMap
{
    public:
        HeightMap(int width, int height);
        ~HeightMap();
        
        void    importFromArray(double* arr);
        void    exportToArray(double* arr);
        
        void    setHeightAt(int x, int y, double height);
        double  getHeightAt(int x, int y);
        
        void 	calculateTypes();
	
	HeightMap getRelativeMap(int px, int py, Rotation rot);
	
	void 	grow();
        
        void displayConsole();
        void displayGUI(int, int, int);
        void displayTypesGUI();
        void display3D();
	
	vector<double>& getHeights();
        vector<int>& getTypes();
        
    private:
        double&                 _at(int x, int y);
        
        vector<double>          _heights;  
        vector<int>		 _types;
        int                     _width;
        int                     _height;
        double                  _min, _max;
        Mat			 _compass;
        Mat			 _arrow;
	bool 			 _mayGrow;
};
    
    
    



#endif
