// AffPoint.h -- 3D Affine points
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

// When spherical coordinates are used below, the meaning of the (theta,phi)
// angles is the following. Assume R is the vector from the origin to a
// given point in space. Assume Rxy is the projection of the vector R onto the
// xy-plane. Then:
//     phi   = angle between R and the z-axis (0 <= phi <= PI)
//     theta = angle from the x-axis to Rxy. Positive angles are counter-
//             clockwise from the x-axis. Normal range: (-PI <= theta <= +PI).
// Notice that this convention is different from the convention used in some
// contexts such as image synthesis where the roles of theta and phi are often
// reversed, probably because theta has historically been used in Image
// Synthesis to characterize an angle of incidence. That is, to describe the
// angle between the outward pointing normal vector to a surface (a local "z"
// axis) and an incoming light direction.

#ifndef AFFPOINT_H
#define AFFPOINT_H

#include <math.h>
#include <iostream>

#include "AffVector.h"

namespace cryph
{

// indices for extracting coordinates from points

const int X = 0;
const int Y = 1;
const int Z = 2;
const int W = 3;

class AffPoint
{
public:
	AffPoint(); // point at origin
	AffPoint(double xx, double yy, double zz=0.0);
	AffPoint(const AffPoint& p);
	AffPoint(const double* p); // assumes x=p[0], y=p[1], z=p[2]
	AffPoint(const float* p); // assumes x=p[0], y=p[1], z=p[2]
	AffPoint(const AffVector& v);
	virtual ~AffPoint();

	AffPoint operator=(const AffPoint& rhs);
	AffPoint operator+=(const AffVector& rhs);
	AffPoint operator+=(const AffPoint& rhs);
	AffPoint operator-=(const AffVector& rhs);
	AffPoint operator*=(double f);
	AffPoint operator/=(double f);
	double	 operator[](int index) const; // read-only indexing
					// see indexing constants above

	AffPoint operator+(const AffPoint& p2) const
				{ return AffPoint(x + p2.x, y + p2.y, z + p2.z); }
	AffPoint operator*(double f) const
				{ return AffPoint (f*x, f*y, f*z); }
	AffPoint operator/(double f) const
				{ return AffPoint (x/f, y/f, z/f); }
	AffVector operator-(const AffPoint& p2) const
				{ return AffVector(x-p2.x, y-p2.y, z-p2.z); }
	AffPoint operator+(const AffVector& v2) const
				{ return AffPoint(x+v2[DX], y+v2[DY], z+v2[DZ]); }
	AffPoint operator-(const AffVector& v2) const
				{ return AffPoint(x-v2[DX], y-v2[DY], z-v2[DZ]); }

	// ---------- General Methods
	// Among other things, the following two methods are useful for interfacing
	// with OpenGL, especially routines like glBufferData. For example, a caller
	// can allocate an array of an appropriate size, then loop over an array
	// of AffPoint instances, invoking the appropriate aCoords method below.
	// For example:
	//     float* buf = new float[3*NUM_POINTS];
	//     for (int i=0 ; i<NUM_POINTS ; i++)
	//         affPointArray[i].aCoords(buf, 3*i);
	//     glBufferData(GL_ARRAY_BUFFER, 3*NUM_POINTS*sizeof(float),
	//                  buf, GL_STATIC_DRAW);
	//     delete [] buf;
	double* aCoords(double* coords, int offset=0) const;
	float* aCoords(float* coords, int offset=0) const;

	void barycentricCoords( // in a plane
				// find the areal Barycentric coordinates
				// of "this" point with respect to:
				const AffPoint& P1, const AffPoint& P2, const AffPoint& P3,
				// returning them in:  (b1+b2+b3 = 1)
				double& b1, double& b2, double& b3) const;
	void barycentricCoords( // on a line
				// find the areal Barycentric coordinates of "this"
				// point with respect to:
				const AffPoint& P1, const AffPoint& P2,
				// returning them in:  (b1+b2 = 1)
				double& b1, double& b2) const;
	bool coincidentWith(const AffPoint& p) const;
	double distanceFromLine(const AffPoint& B, const AffVector& u) const;
	double distanceFromOrigin() const;
	double distanceSquaredFromLine(const AffPoint& B, const AffVector& u) const;
	double distanceSquaredFromOrigin() const;
	double distanceSquaredTo(const AffPoint& p) const;
	double distanceTo(const AffPoint& p) const;
	double normalize(); // move point along line through origin until
	                    // it lies on the unit sphere
	// Next 4: extract coords, represent in projective space using
	// provided "w" -> indicated array
	double* pCoords(double* coords, double w) const;
	double* pCoords(double* coords, double w, int offset) const;
	float* pCoords(float* coords, double w) const;
	float* pCoords(float* coords, double w, int offset) const;
	void swizzle(char xyz[3]);
	void toCylindrical(double& r, double& theta, double& z) const;
	void toSpherical(double& rho, double& theta, double& phi) const;

	// ---------- General Class Methods
	static AffPoint centroid(const AffPoint p[], int nPoints);
	static AffPoint fromBarycentricCoords( // in a plane
							const AffPoint& P1, const AffPoint& P2,
							const AffPoint& P3,
							double b1, double b2, double b3)
						{ return b1*P1 + b2*P2 + b3*P3; }
	static AffPoint fromBarycentricCoords( // on a line
							const AffPoint& P1, const AffPoint& P2,
							double b1, double b2)
						{ return b1*P1 + b2*P2; }
	static AffPoint fromCylindrical(double r, double theta, double z)
						{ return AffPoint( r*cos(theta), r*sin(theta), z ); }
	static AffPoint fromSpherical(double rho, double theta, double phi)
						{ return AffPoint(rho*sin(phi)*cos(theta),
										  rho*sin(phi)*sin(theta), rho*cos(phi) ); }
	static double getCoincidenceTolerance() { return AffPoint::sCoincidenceTol; }
	// maxOffsetInDirection: find the point in the given buffer that is farthest
	// away in the given direction. If that point is 'P', the function return
	// value is "(P-ref).dir". If there is exactly one such farthest point, then
	// the indices i1 and i2 will both contain the index of 'P' in 'buf'. If several
	// points tie, then i1 and i2 are the indices of the first succession of adjacent
	// points all at that offset. If there are multiple sets of adjacent tieing points,
	// only the indices of the first are returned.
	static double maxOffsetInDirection(
					const AffPoint& ref, const AffVector& dir,
					const AffPoint buf[], int bufSize,
					int& index1, int& index2);
	static double ratio(const AffPoint& a, const AffPoint& b, const AffPoint& c);
	static void setCoincidenceTolerance(double tol);

    // ---------- Global constants

	// Special Points
	static const AffPoint		origin;
	static const AffPoint		xAxisPoint;
	static const AffPoint		yAxisPoint;
	static const AffPoint		zAxisPoint;

	// The coordinates are public, but it is usually best to use
	// the public methods, using public access to the following only
	// for convenient overwriting of the coordinates of a specific point.
	double	x;
	double	y;
	double	z;

private:
	static double	sCoincidenceTol;
};

std::ostream&		operator<<(std::ostream& os, const AffPoint& p);
std::istream&		operator>>(std::istream& is, AffPoint& p);

static AffPoint operator*(double f, const AffPoint& p)
	{ return AffPoint(f*p[X], f*p[Y], f*p[Z]); }
}

#endif
