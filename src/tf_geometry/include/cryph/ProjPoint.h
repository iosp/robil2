// ProjPoint.h -- 4D Projective Space points
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef PROJPOINT_H
#define PROJPOINT_H

#include "AffPoint.h"

namespace cryph
{

class ProjPoint
{
public:

	// --------- basic constructors
	ProjPoint(); // 4D point at origin
	ProjPoint(const ProjPoint& p);
	ProjPoint(const AffPoint& p, double w=1.0);
	ProjPoint(const double* p);	// 4-component projective space point
	ProjPoint(const float* p);		// 4-component projective space point
	ProjPoint(double xx, double yy, double zz=0.0, double ww=1.0);

	// --------- destructor
	virtual ~ProjPoint();

	// --------- operators as methods
	ProjPoint	operator=(const ProjPoint& rhs);
	ProjPoint	operator+=(const ProjPoint& rhs);
	ProjPoint	operator*=(double f);
	ProjPoint	operator/=(double f);
	double		operator[](int index) const; // read-only indexing
				// see indexing constants above

	// --------- operators
	ProjPoint	operator+(const ProjPoint& p2) const
				{ return ProjPoint(x+p2.x, y+p2.y, z+p2.z, w+p2.w); }
	ProjPoint	operator-(const ProjPoint& p2) const
				{ return ProjPoint(x-p2.x, y-p2.y, z-p2.z, w-p2.w); }
	ProjPoint	operator*(double f) const
				{ return ProjPoint(f*x, f*y, f*z, f*w); }
	ProjPoint	operator/(double f) const
				{ return ProjPoint(x/f, y/f, z/f, w/f); }

	// ---------- General Methods
					// extract affine coordinates ...
	double*		aCoords(double coords[], int offset=0) const; // ... into double[]
	float*		aCoords(float coords[], int offset=0) const; // ... into float[]
	void		aCoords(AffPoint& aPnt) const    // ... into an AffPoint
				{ aPnt = AffPoint(x/w, y/w, z/w); }
	AffPoint	aCoords() const    // ... into an AffPoint
				{ return AffPoint(x/w, y/w, z/w); }
	double*		pCoords(double* coords, int offset=0) const;
	float*		pCoords(float* coords, int offset=0) const;
	void		swizzle(char xyz[3]);

	// Coordinates are public, but you should try to use direct access
	// to these only for simple alteration of coordinates. Otherwise
	// use public methods.
	double	x;
	double	y;
	double	z;
	double	w;
};

std::ostream&	operator<<(std::ostream& os, const ProjPoint& p);
std::istream&	operator>>(std::istream& is, ProjPoint& p);

static ProjPoint operator*(double f, const ProjPoint& p)
	{ return ProjPoint(f*p[X],f*p[Y],f*p[Z],f*p[W]); }
}

#endif
