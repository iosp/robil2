#ifndef HEIGHTMAP__H
#define HEIGHTMAP__H
#include "helpermath.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "heightmap_object.h"
// #include <per/roadLanes.h>
// #include <per/lane.h>
// using namespace per;
using namespace std;
using namespace cv;

class HeightMap: public HeightmapObject
{
    public:
        HeightMap(int width, int height, per::configConfig *p);
        ~HeightMap();
        

    double calc_height(int, int, std::vector<int>);
    double calc_slope(int, int, std::vector<int>);
	/**
	 * This function attempts to (for now) classify each map cell as
	 * passable, obstacle or unknown. Expected to work at lightning speed.
	 */
    void calculateTypes();//Vec3D position, Rotation myRot);
	

	HeightMap deriveMap(int px, int py, Rotation rot);
	HeightMap deriveMiniMap(int px, int py, Rotation rot);
        
private:

};
    
    
    



#endif
