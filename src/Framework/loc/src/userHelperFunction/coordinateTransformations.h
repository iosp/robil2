#include <fstream>
#include <string>
#include <stdlib.h>     /* atof */
#include "../component/gps_calculator.h"


#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>

using namespace std;

sensor_msgs::NavSatFix read_file(char *fname,char *strtim)
{
  /*
   * This function searches for the ~/.ros/gps_init.txt file with the initial coordinates.
   * The function will search for the file, and if it exists it will take the alltitude, 
   * longitude, latitude data, and returns a NavSatFix variable with the data in the file.
   */
  sensor_msgs::NavSatFix coor;
  coor.altitude = -1;
  coor.longitude = -1;
  coor.latitude = -1;
  string line;
  //cout << "trying to open file: " << endl << fname << endl;
  ifstream myfile (fname);//load file
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      if (line.compare(0,4,"Time") == 0) //Search for the line of 'time of creation' of the file.
      {
	  char buffer[20];
	  std::size_t length = line.copy(buffer,20,5);
	  buffer[length]='\0';
	  int i=0;
	  while (buffer[i]!=0)
	    strtim[i++] = buffer[i];
	  strtim[i]=0;
      }
      if (line.compare(0,8,"latitude") == 0) //Search for the latitude line
      {
	  char buffer[20];
	  std::size_t length = line.copy(buffer,20,9);
	  buffer[length]='\0';
	  coor.latitude = atof(buffer);
      }
      if (line.compare(0,9,"longitude") == 0) //Search for the longitude line 
      {
	  char buffer[20];
	  std::size_t length = line.copy(buffer,20,10);
	  buffer[length]='\0';
	  coor.longitude = atof(buffer);
      }
      if (line.compare(0,8,"altitude") == 0) //Search for the altitude line
      {
	  char buffer[20];
	  std::size_t length = line.copy(buffer,20,9);
	  buffer[length]='\0';
	  coor.altitude = atof(buffer);
      }
    }
    myfile.close();
  }

  else 
  {
    cout << "Unable to open temp file\n"; 
  }
  return coor;
}

inline geometry_msgs::Point geo2xy(sensor_msgs::NavSatFix point, double bearing=0)
{
  /*
   * This function receives a NavSatFix point opens the gps_init file and calculates the (x,y) values
   * from between these two points.
   * The return value is a geometry_msgs point variable that contains these (x,y) values.
   * 
   * If Couldn't read the file, or if the data is corupted, the variable will return (-1,-1,-1)
   */
  geometry_msgs::Point var;
  var.x = -1;
  var.y = -1;
  var.z = -1;
  
  char strtim[20];
  
  char dir[100];
  sprintf(dir,"%s/.ros/gps_init.txt",getenv("HOME"));
  sensor_msgs::NavSatFix init_coor = read_file(dir,strtim);

  if (init_coor.altitude == -1 || init_coor.longitude == -1 || init_coor.latitude == -1)
    return var; // If coudn't open file return (-1,-1,-1)

  double d = calcDistance(point,init_coor);
  double theta = calcBearing(init_coor,point);
  var.x = d * cos(theta);
  var.y = d * sin(theta);
  
  var.z = point.altitude - init_coor.altitude;
  return var;
}
inline sensor_msgs::NavSatFix xy2geo(geometry_msgs::Point point)
{
  /*
   * This function receives an (x,y) point opens the gps_init file and calculates the (latitude,longitude) values
   * from point.
   * The return value is a sensor_msgs NavSatFix variable that contains these (latitude,longitude) values.
   * 
   * If Couldn't read the file, or if the data is corupted, the variable will return (0,0,0)
   */
  
  sensor_msgs::NavSatFix var;
  char strtim[20];
  char dir[100];
  sprintf(dir,"%s/.ros/gps_init.txt",getenv("HOME"));
  sensor_msgs::NavSatFix init_coor = read_file(dir,strtim);
  if (init_coor.altitude == -1 || init_coor.longitude == -1 || init_coor.latitude == -1)
    return var; // If coudn't open file return (-1,-1,-1)

  double d = sqrt(point.x * point.x + point.y * point.y);
  double theta = atan2(point.y,point.x);
  
  var = calcLatLon(init_coor,d,theta);
  var.altitude = point.z - init_coor.altitude;
  return var;
}
