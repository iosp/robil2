/*
 * This is an example of using the coordinateTransformations header file.
 * ======================================================================
 * 
 * All you have to do is to call the function geo2xy(point)
 * geo2xy(point) which receives as input NavSatFix point.
 * geo2xy(point) returns as output a geometry_msgs point (x,y,z).
 * 
 */


#include "coordinateTransformations.h" //You must include the coordinateTransformations header!
#include <iostream>
using namespace std;
int main()
{
  // Example 1: get x,y value from a latitude,longitude value
  sensor_msgs::NavSatFix point; //create a NavSatFix point and load it with data
  point.altitude = 0;
  point.latitude = 31.2624697963104;
  point.longitude = 34.8038214171071;
  geometry_msgs::Point result1; //create a geometry_msgs point to save the returned value.
  
  result1 = geo2xy(point); // call the function geo2xy()
  cout << result1 << endl;
  
  // Example 2: get latitude,longitude values from x,y values
  geometry_msgs::Point point2;
  point2.x = 30; //[m]
  point2.y = 20; //[m]
  point2.z = 0;  //[m]
  
  sensor_msgs::NavSatFix result2;
  result2 = xy2geo(point2);
  cout << std::setprecision(15) << result2.altitude << endl << result2.latitude << endl << result2.longitude << endl;
  return 0;
}
