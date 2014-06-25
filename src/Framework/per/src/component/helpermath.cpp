#include "helpermath.h"
#include <string>
#include <cstdio>
#include <cmath>
using std::string;


Vec3D GetUpVector(double x, double y, double z, double w) 
{
  return Vec3D( 2 * (x * z + w * y),
		  2 * (y * z - w * x),
		  1 - 2 * (x * x + y * y));
}
 
Vec3D GetRightVector(double x, double y, double z, double w) 
{
    return Vec3D( 2 * (x * y - w * z),
                    1 - 2 * (x * x + z * z),
                    2 * (y * z + w * x));
}
 
Vec3D GetFrontVector(double x, double y, double z, double w) 
{
    return Vec3D( 1 - 2 * (y * y + z * z),
                    2 * (x * y + w * z),
                    2 * (x * z - w * y));
}

Quaternion GetFromRPY(Rotation rot)
{
  double phi, the, psi;

  phi = rot.roll / 2.0;
  the = rot.pitch / 2.0;
  psi = rot.yaw / 2.0;
  
  double w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
  double x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
  double y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
  double z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
  return Quaternion(x,y,z,w);
  
}

Rotation GetRotation(Quaternion q)
{
  Rotation rot;
  
  double squ;
  double sqx;
  double sqy;
  double sqz;

  //Normalize();

  squ = q.w * q.w;
  sqx = q.x * q.x;
  sqy = q.y * q.y;
  sqz = q.z * q.z;

  // Roll
  rot.roll = atan2(2 * (q.y*q.z + q.w*q.x), squ - sqx - sqy + sqz);

  // Pitch
  double sarg = -2 * (q.x*q.z - q.w * q.y);
  rot.pitch = sarg <= -1.0 ? -0.5*M_PI : (sarg >= 1.0 ? 0.5*M_PI : asin(sarg));
  
  // Yaw
  rot.yaw = atan2(2 * (q.x*q.y + q.w*q.z), squ + sqx - sqy - sqz);

  return rot;
}

