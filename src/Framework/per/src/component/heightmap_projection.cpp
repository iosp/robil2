#include "heightmap_projection.h"


void ProjectLaserRange(HeightMap* map, Vec3D myRight, Vec3D myFront, Vec3D myPos, double range, double scanAngle)
{
  if (range < 2) return;
    double rcos_a = range*cos(scanAngle);
    double rsin_a = range*sin(scanAngle);

    Vec3D pos = myPos.add(myFront.multiply(rcos_a).add(myRight.multiply(rsin_a)));
    map->setAbsoluteHeightAt((int)(5*pos.x), (int)(5*pos.y), (pos.z));   
}

void ProjectLaserRange(HeightMap* map, sensor_msgs::PointCloud *points, double height_of_tracks)
{
    double min_dist_for_detection = 1;
    for (int i = 0; i < points->points.size(); i++)
    {
//        double range = points->points[i].x * points->points[i].x + points->points[i].y * points->points[i].y; //range squared
//        if (range < min_dist_for_detection * min_dist_for_detection) continue;
        map->setAbsoluteHeightAt((int)(5*points->points[i].x), (int)(5*points->points[i].y), (points->points[i].z) - height_of_tracks);
    }
}
