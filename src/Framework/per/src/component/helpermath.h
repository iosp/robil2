#ifndef HELPERMATH__H
#define HELPERMATH__H
#include <string>
#include <cstdio>
#include <cmath>
using std::string;

class Vec3D
{
    public:
        Vec3D(double x, double y, double z) : x(x), y(y), z(z) { }
        Vec3D() { x = y = z = 0; }
        Vec3D multiply(double r) { return Vec3D(x*r, y*r, z*r); }        
        Vec3D add(Vec3D other) { return Vec3D(x+other.x, y+other.y, z+other.z); }
        double x,y,z;
	string toString() 
	{ 
	  char buf[50];
	  sprintf(buf, "V[%f %f %f]", x, y, z);
	  return string(buf);
	}
};

class Vec2D
{
  public:
        Vec2D(double x, double y) : x(x), y(y) { }
        Vec2D() { x = y = 0; }
        Vec2D multiply(double r) { return Vec2D(x*r, y*r); }        
        Vec2D add(Vec2D other) { return Vec2D(x+other.x, y+other.y); }
        double length() { return sqrt(x*x+y*y); }
        Vec2D normalize() { return Vec2D(x/length(), y/length()); };
        double x,y;
	string toString() 
	{ 
	  char buf[50];
	  sprintf(buf, "V[%f %f]", x, y);
	  return string(buf);
	}
};

class Rotation
{
  public:
    Rotation(double roll, double pitch, double yaw) : roll(roll), pitch(pitch), yaw(yaw) {}
    Rotation() : roll(0), pitch(0), yaw(0) {}
    Rotation add(Rotation other) { return Rotation(roll+other.roll, pitch+other.pitch, yaw+other.yaw); }
    double roll, pitch, yaw;
    string toString() 
    { 
      char buf[50];
      sprintf(buf, "R[%f %f %f]", roll, pitch, yaw);
      return string(buf);
    }
    
};

class Quaternion
{
  public:
    Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
    Quaternion() : x(0), y(0), z(0), w(0) {}
    double x,y,z,w;
    string toString() 
    { 
      char buf[50];
      sprintf(buf, "Q[%f %f %f %f]", x, y, z, w);
      return string(buf);
    }
};


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


#endif