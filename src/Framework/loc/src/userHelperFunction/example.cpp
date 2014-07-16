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

using namespace std;
int main()
{
  
  sensor_msgs::NavSatFix point; //create a NavSatFix point and load it with data
  point.altitude = 0;
  point.latitude = 31.2621;
  point.longitude = 34.8035;
  geometry_msgs::Point var0; //create a geometry_msgs point to save the returned value.
  
  var0 = geo2xy(point); // call the function geo2xy()
  cout << var0 << endl;
}
