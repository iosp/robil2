// AffVector.h -- 3D vectors associated with a 3D affine space
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef AFFVECTOR_H
#define AFFVECTOR_H

#include <math.h>
#include <iostream>

namespace cryph
{
class AffPoint;

// indices for extracting components from vectors

const int DX = 0;
const int DY = 1;
const int DZ = 2;
const int DW = 3;

class AffVector
{
public:
	AffVector(); // zero vector
	AffVector(double Dx, double Dy, double Dz=0.0);
	AffVector(const AffVector& v);
	AffVector(const AffPoint& p);
	AffVector(const double xyz[]); // assumes xyz contains 3D components
	AffVector(const float xyz[]);  // assumes xyz contains 3D components
	virtual ~AffVector();

	AffVector operator=(const AffVector& rhs);
	AffVector operator+=(const AffVector& rhs);
	AffVector operator-=(const AffVector& rhs);
	AffVector operator*=(double f);
	AffVector operator/=(double f);
	double operator[](int index) const; // read-only indexing
					// see indexing constants above

	AffVector	operator+(const AffVector& v2) const
				{ return AffVector(dx+v2.dx , dy+v2.dy , dz+v2.dz); }
	AffVector	operator-(const AffVector& v2) const
				{ return AffVector(dx-v2.dx , dy-v2.dy , dz-v2.dz); }
	AffVector	operator-() const { return AffVector(-dx, -dy, -dz); }
	AffVector	operator*(double f) const
				{ return AffVector(f*dx , f*dy , f*dz); }
	AffVector	operator/(double f) const
				{ return AffVector(dx/f , dy/f , dz/f); }

	// ---------- General Methods
	void arbitraryNormal(AffVector& normal) const;
	AffVector cross(const AffVector& rhs) const;
	void decompose(const AffVector& arbitraryVector,
					AffVector& parallel, AffVector& perpendicular) const;
	double dot(const AffVector& rhs) const
				{ return dx*rhs.dx + dy*rhs.dy + dz*rhs.dz; }
	double length() const { return sqrt(lengthSquared()); }
	double lengthSquared() const { return dx*dx + dy*dy + dz*dz; }
	double maxAbsComponent(int& componentIndex) const;
	double minAbsComponent(int& componentIndex) const;
	double normalize();
	double normalizeToCopy(AffVector& normalizedCopy) const;
	bool parallelTo(const AffVector& v) const;

	// Among other things, the following two methods are useful for interfacing
	// with OpenGL, especially routines like glBufferData. For example, a caller
	// can allocate an array of an appropriate size, then loop over an array
	// of AffVector instances, invoking the appropriate vComponents method.
	// For example:
	//     float* buf = new float[3*NUM_VECTORS];
	//     for (int i=0 ; i<NUM_VECTORS ; i++)
	//         affVectorArray[i].vComponents(buf, 3*i);
	//     glBufferData(GL_ARRAY_BUFFER, 3*NUM_VECTORS*sizeof(float),
	//                  buf, GL_STATIC_DRAW);
	//     delete [] buf;
	double* vComponents(double* components, int offset=0) const;
	float* vComponents(float* components, int offset=0) const;

	// ---------- Class Methods
	// The following two methods assume U and W (or V and W) have
	// values (even if all zero). W gets normalized; the component
	// of U (V) perpendicular to W becomes U. Finally V (or U) is
	// computed from an appropriate cross product. If W is zero
	// vector, then (xu, yu, zu) are copied into U, V, and W. If
	// the component of U (V) perpendicular to W is zero, then an
	// arbitrary vector perpendicular to W is created and used.)
	static void coordinateSystemFromUW(AffVector& U, AffVector& V,
					AffVector& W);
	static void coordinateSystemFromVW(AffVector& U, AffVector& V,
					AffVector& W);

	static AffVector cross(const AffVector& v1, const AffVector& v2);
	static double dot(const AffVector& v1, const AffVector& v2)
					{ return v1.dx*v2.dx + v1.dy*v2.dy + v1.dz*v2.dz; }

	// Special Vectors
	static const AffVector xu;
	static const AffVector yu;
	static const AffVector zu;
	static const AffVector zeroVector;

	// The vector components are public, but it is best to use the
	// other public methods whenever possible. Only use direct access
	// to these variables for ease of overwriting some specific
	// component.
	double	dx;
	double	dy;
	double	dz;
};

std::ostream&	operator<<(std::ostream& os, const AffVector& v);
std::istream&	operator>>(std::istream& is, AffVector& v);

static AffVector operator*(double f, const AffVector& v)
	{ return AffVector(f*v[DX] , f*v[DY] , f*v[DZ]); }
}

#endif
