#ifndef HEIGHTMAP_PROJECTION__H
#define HEIGHTMAP_PROJECTION__H

#include <cmath>
#include "helpermath.h"
#include "heightmap.h"
#include "sensor_msgs/PointCloud.h"
void ProjectLaserRange(HeightMap* map, Vec3D myRight, Vec3D myFront, Vec3D myPos, double range, double scanAngle);
void ProjectLaserRange(HeightMap* map, sensor_msgs::PointCloud *points, double height_of_tracks);

#endif
