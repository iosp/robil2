#ifndef HEIGHTMAP_PROJECTION__H
#define HEIGHTMAP_PROJECTION__H

#include <cmath>
#include "helpermath.h"

void ProjectLaserRange(HeightMap* map, Vec3D myRight, Vec3D myFront, Vec3D myPos, double range, double scanAngle)
{
    double rcos_a = range*cos(scanAngle);
    double rsin_a = range*sin(scanAngle);
    
    Vec3D pos = myPos.add(myFront.multiply(rcos_a).add(myRight.multiply(rsin_a)));
    map->setHeightAt((int)(5*pos.x), (int)(5*pos.y), (pos.z));   
}


#endif
