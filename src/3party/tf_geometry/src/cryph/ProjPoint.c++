// ProjPoint.c++ -- 4D Projective Space points
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <stdlib.h>

#include <cryph/Inline.h>
#include <cryph/ProjPoint.h>

using namespace std;

namespace cryph
{

ProjPoint::ProjPoint() : x(0.0), y(0.0), z(0.0), w(1.0)
{
}

ProjPoint::ProjPoint(const ProjPoint& p) : x(p.x), y(p.y), z(p.z), w(p.w)
{
}

ProjPoint::ProjPoint(const AffPoint& p, double W) :
	x(W*p[X]), y(W*p[Y]), z(W*p[Z]), w(W)
{
}

ProjPoint::ProjPoint(const double p[]) : x(p[0]), y(p[1]), z(p[2]), w(p[3])
{
}

ProjPoint::ProjPoint(const float p[]) : x(p[0]), y(p[1]), z(p[2]), w(p[3])
{
}

ProjPoint::ProjPoint(double xx, double yy, double zz, double ww) :
	x(xx), y(yy), z(zz), w(ww)
{
}

ProjPoint::~ProjPoint()
{
}

double* ProjPoint::aCoords(double coords[], int offset) const
{
	// extract affine coords and place into a double precision array

	coords[offset  ] = x/w;
	coords[offset+1] = y/w;
	coords[offset+2] = z/w;
	return coords;
}

float* ProjPoint::aCoords(float coords[], int offset) const
{
	// extract affine coords and place into a float array

	coords[offset  ] = (float)(x/w);
	coords[offset+1] = (float)(y/w);
	coords[offset+2] = (float)(z/w);
	return coords;
}

ProjPoint ProjPoint::operator=(const ProjPoint& rhs)
{
	x = rhs.x; y = rhs.y ; z = rhs.z; w = rhs.w;
	return *this;
}

ProjPoint ProjPoint::operator+=(const ProjPoint& rhs)
{
	x += rhs.x; y += rhs.y ; z += rhs.z; w += rhs.w;
	return *this;
}

ProjPoint ProjPoint::operator*=(double f)
{
	x *= f; y *= f ; z *= f; w *= f;
	return *this;
}

ProjPoint ProjPoint::operator/=(double f)
{
	x /= f; y /= f ; z /= f; w /= f;
	return *this;
}

static const double unspecifiedValue = -123.456;
double ProjPoint::operator[](int index) const
{
	// read-only indexing

	switch (index)
	{
		case X:
			return x;
		case Y:
			return y;
		case Z:
			return z;
		case W:
			return w;
	}

	return unspecifiedValue;
}

ostream& operator<<(ostream& os, const ProjPoint& p)
{
	os << p[X] << ' ' << p[Y] << ' ' << p[Z] << ' ' << p[W];
	return os;
}

istream& operator>>(istream& is, ProjPoint& p)
{
	double x, y, z, w;
	is >> x >> y >> z >> w;
	p = ProjPoint(x,y,z,w);

	return is;
}

double* ProjPoint::pCoords(double* coords, int offset) const
{
	coords[offset  ] = x;
	coords[offset+1] = y;
	coords[offset+2] = z;
	coords[offset+3] = w;
	return coords;
}

float* ProjPoint::pCoords(float* coords, int offset) const
{
	coords[offset  ] = static_cast<float>(x);
	coords[offset+1] = static_cast<float>(y);
	coords[offset+2] = static_cast<float>(z);
	coords[offset+3] = static_cast<float>(w);
	return coords;
}

void ProjPoint::swizzle(char xyz[3])
{
	double xyzC[4], xyzO[4];
	pCoords(xyzC);
	pCoords(xyzO);
	for (int i=0 ; i<3 ; i++)
		switch (xyz[i])
		{
			case 'x': xyzC[i] = xyzO[0]; break;
			case 'y': xyzC[i] = xyzO[1]; break;
			case 'z': xyzC[i] = xyzO[2]; break;
			case 'X': xyzC[i] = -xyzO[0]; break;
			case 'Y': xyzC[i] = -xyzO[1]; break;
			case 'Z': xyzC[i] = -xyzO[2]; break;
			default:
				; // just ignore
		}
	x = xyzC[0];
	y = xyzC[1];
	z = xyzC[2];
}

}
