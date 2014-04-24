#ifndef HEIGHTMAP__H
#define HEIGHTMAP__H

#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/**
    Robil-2 heightmap for the mapping component.
    Designed according to specs.
*/

class HeightMap
{
    public:
        HeightMap(int width, int height);
        ~HeightMap();
        
        void    importFromArray(double* arr);
        void    exportToArray(double* arr);
        
        void    setHeightAt(int x, int y, double height);
        double  getHeightAt(int x, int y);
        
        
        
        void displayConsole();
        void displayGUI(int, int, int);
        void display3D();
        
        
    private:
        double&                 _at(int x, int y);
        
        vector<double>          _heights;  
        int                     _width;
        int                     _height;
        double                  _min, _max;
	Mat			 _compass;
        Mat			 _arrow;
};
    
    
    



#endif
