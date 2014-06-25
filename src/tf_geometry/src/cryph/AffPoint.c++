// AffPoint.c++ -- 3D Affine points
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <stdlib.h>

#include <cryph/AffPoint.h>
#include <cryph/AffVector.h>
#include <cryph/Inline.h>
#include <cryph/Tolerances.h>

namespace cryph
{

// ---------- Global constants

// Special Points
const AffPoint AffPoint::origin     = AffPoint(0.0 , 0.0 , 0.0);
const AffPoint AffPoint::xAxisPoint = AffPoint(1.0 , 0.0 , 0.0);
const AffPoint AffPoint::yAxisPoint = AffPoint(0.0 , 1.0 , 0.0);
const AffPoint AffPoint::zAxisPoint = AffPoint(0.0 , 0.0 , 1.0);

// Class variables

double AffPoint::sCoincidenceTol = BasicDistanceTol;

AffPoint::AffPoint() : x(0), y(0), z(0)
{
}

AffPoint::AffPoint(double xx, double yy, double zz) : x(xx), y(yy), z(zz)
{
}

AffPoint::AffPoint(const AffPoint& p) : x(p.x), y(p.y), z(p.z)
{
}

AffPoint::AffPoint(const double* p) : x(p[0]), y(p[1]), z(p[2])
{
}

AffPoint::AffPoint(const float* p) : x(p[0]), y(p[1]), z(p[2])
{
}

AffPoint::AffPoint(const AffVector& v) : x(v[DX]), y(v[DY]), z(v[DZ])
{
}

AffPoint::~AffPoint()
{
}

double* AffPoint::aCoords(double* coords, int offset) const
{
	// extract affine coords and place into a double precision array

	coords[offset] = x;
	coords[offset+1] = y;
	coords[offset+2] = z;
	return coords;
}

float* AffPoint::aCoords(float* coords, int offset) const
{
	// extract affine coords and place into a single precision (float) array

	coords[offset] = static_cast<float>(x);
	coords[offset+1] = static_cast<float>(y);
	coords[offset+2] = static_cast<float>(z);
	return coords;
}

#define TESTING 0

void AffPoint::barycentricCoords( // in a plane
	// find the areal Barycentric coordinates of "this" point with respect to:
	const AffPoint& P1, const AffPoint& P2, const AffPoint& P3,
	// returning them in:  (b1+b2+b3 = 1)
	double& b1, double& b2, double& b3) const
{
	AffVector nToTriangle = (P2-P1).cross(P3-P1);
	// b1:
	AffVector ref = nToTriangle.cross(P3-P2);
	ref.normalize();
	double triangleHeight  = (P1 - P2).dot(ref);
	double signedHeightFromPoint = (*this - P2).dot(ref);
	b1 = signedHeightFromPoint/triangleHeight;

	// b2:
	ref = nToTriangle.cross(P1-P3);
	ref.normalize();
	triangleHeight  = (P2 - P3).dot(ref);
	signedHeightFromPoint = (*this - P3).dot(ref);
	b2 = signedHeightFromPoint/triangleHeight;

	// b3:
#if TESTING
	ref = nToTriangle.cross(P2 - P1);
	ref.normalize();
	triangleHeight  = (P3 - P1).dot(ref);
	signedHeightFromPoint = (*this - P1).dot(ref);
	b3 = signedHeightFromPoint/triangleHeight;
#else
	b3 = 1.0 - b1 - b2;
#endif
}

void AffPoint::barycentricCoords( // on a line
	// find the areal Barycentric coordinates of "this" point with respect to:
	const AffPoint& P1, const AffPoint& P2,
	// returning them in:  (b1+b2 = 1)
	double& b1, double& b2) const
{
	AffVector u = P2 - P1;
	double dist = u.normalize();
	if (fabs(dist) < BasicDistanceTol)
	{
		b1 = 1.0; b2 = 0.0;
		return;
	}
	b2 = u.dot(*this - P1) / dist;
	b1 = 1.0 - b2;
}

AffPoint AffPoint::centroid(const AffPoint p[], int nPoints)
{
	AffPoint	sum(p[0]);
	for (int i=1 ; i<nPoints ; i++)
		sum = sum + p[i];
	return (sum / (double)nPoints);
}

bool AffPoint::coincidentWith(const AffPoint& p) const
{
	return (this->distanceTo(p) < AffPoint::sCoincidenceTol);
}

double AffPoint::distanceFromLine(const AffPoint& B, const AffVector& u) const
{
	return sqrt( distanceSquaredFromLine(B,u) );
}

double AffPoint::distanceFromOrigin() const
{
	return sqrt( distanceSquaredFromOrigin() );
}

double AffPoint::distanceSquaredFromLine(const AffPoint& B, const AffVector& u) const
{
	// Pythagorean theorem approach: c^2 = a^2 + b^2. The distance squared
	// we seek is b^2:
	AffVector toPoint = *this - B;
	double cSquared = toPoint.dot(toPoint);
	AffVector dir;
	u.normalizeToCopy(dir);
	double aSquared = dir.dot(toPoint);
	aSquared *= aSquared;
	double distSquared = cSquared - aSquared;
	if (distSquared < 0.0)
		// must be a tiny number that really ought to be 0.0
		distSquared = 0.0;
	return distSquared;
}

double AffPoint::distanceSquaredFromOrigin() const
{
	return x*x + y*y + z*z;
}

double AffPoint::distanceSquaredTo(const AffPoint& p) const
{
	double t = (x - p.x);
	double s = t*t;
	t = (y - p.y);
	s += t*t;
	t = (z - p.z);
	s += t*t;

	return s;
}

double AffPoint::distanceTo(const AffPoint& p) const
{
	return sqrt( distanceSquaredTo(p) );
}

double AffPoint::maxOffsetInDirection(const AffPoint& ref, const AffVector& dir,
			const AffPoint buf[], int bufSize, int& index1, int& index2)
{
	index1 = index2 = -1;
	if (bufSize < 1)
		return 0.0;

	index1 = index2 = 0;
	double offset = dir.dot(buf[0] - ref);
	for (int i=1 ; i<bufSize ; i++)
	{
		double t = dir.dot(buf[i] - ref);
		if (equalScalars(t,offset,BasicDistanceTol))
		{
			if (i == index2+1)
				index2++;
		}
		else if (t > offset)
		{
			index1 = index2 = i;
			offset = t;
		}
	}
	return offset;
}

double AffPoint::normalize()
{
	double f = distanceFromOrigin();
	if (f < AffPoint::sCoincidenceTol)
		return 0.0;

	x /= f; y /= f; z /= f;
	return f;
}

AffPoint AffPoint::operator=(const AffPoint& rhs)
{
	x = rhs.x; y = rhs.y ; z = rhs.z;
	return *this;
}

AffPoint AffPoint::operator+=(const AffVector& rhs)
{
	x += rhs[DX]; y += rhs[DY] ; z += rhs[DZ];
	return *this;
}

AffPoint AffPoint::operator+=(const AffPoint& rhs)
{
	x += rhs.x; y += rhs.y ; z += rhs.z;
	return *this;
}

AffPoint AffPoint::operator-=(const AffVector& rhs)
{
	x -= rhs[DX]; y -= rhs[DY] ; z -= rhs[DZ];
	return *this;
}

AffPoint AffPoint::operator*=(double f)
{
	x *= f; y *= f ; z *= f;
	return *this;
}

AffPoint AffPoint::operator/=(double f)
{
	x /= f; y /= f ; z /= f;
	return *this;
}

static const double unspecifiedValue = -123.456;
double AffPoint::operator[](int index) const
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
			return 1.0; // w == 1
	}

	return unspecifiedValue;
}

// operators that are external functions, not methods

std::ostream& operator<<(std::ostream& os, const AffPoint& p)
{
	os << p[X] << ' ' << p[Y] << ' ' << p[Z];
	return os;
}

std::istream& operator>>(std::istream& is, AffPoint& p)
{
	double x, y, z;
	is >> x >> y >> z;
	p = AffPoint(x,y,z);

	return is;
}

double* AffPoint::pCoords(double* coords, double w) const
{
	return pCoords(coords, w, 0);
}

double* AffPoint::pCoords(double* coords, double w, int offset) const
{
	// extract coords and represent in projective space using the provided "w" 
	// place into a double precision array

	coords[offset] = w*x;
	coords[offset+1] = w*y;
	coords[offset+2] = w*z;
	coords[offset+3] = w;
	return coords;
}

float* AffPoint::pCoords(float* coords, double w) const
{
	return pCoords(coords, w, 0);
}

float* AffPoint::pCoords(float* coords, double w, int offset) const
{
	// extract coords and represent in projective space using the provided "w"
	// place into a single precision array

	coords[offset] = static_cast<float>(w*x);
	coords[offset+1] = static_cast<float>(w*y);
	coords[offset+2] = static_cast<float>(w*z);
	coords[offset+3] = static_cast<float>(w);
	return coords;
}

double AffPoint::ratio(const AffPoint& a, const AffPoint& b, const AffPoint& c)
{
	AffVector u = c - a;
	if (u.normalize() > 0.0)
	{
		double denom = AffVector::dot((c-b),u);
		if (fabs(denom) > BasicDistanceTol)
			return AffVector::dot((b-a),u) / denom;
	}
	return 0.0;
}

void AffPoint::setCoincidenceTolerance(double tol)
{
	if (tol >= 0.0) // really better be strictly > 0!!!
		AffPoint::sCoincidenceTol = tol;
}

void AffPoint::swizzle(char xyz[3])
{
	double xyzC[3], xyzO[3];
	aCoords(xyzC);
	aCoords(xyzO);
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

void AffPoint::toCylindrical(double& r, double& theta, double& z) const
{
	z = this->z;
	theta = atan2(this->y, this->x);
	r = sqrt( (this->x)*(this->x) + (this->y)*(this->y) );
}

void AffPoint::toSpherical(double& rho, double& theta, double& phi) const
{
	AffVector	v(*this); // "v" is the vector from the origin to "this" point
	rho = v.normalize();
	if (rho < BasicDistanceTol)
		// the point is at the origin, hence:
		rho = theta = phi = 0.0;
	else
	{
		AffVector	vPerp(v[DX], v[DY], 0.0);
		double r = vPerp.normalize();
		if (r < BasicDistanceTol)
		{
			// point is on the z-axis ==> phi=0 or PI; theta is arbitrary:
			theta = 0.0;
			if (this->z > 0.0)
				phi = 0.0;
			else
				phi = M_PI;
		}
		else
		{
			theta = atan2(vPerp[DY],vPerp[DX]);
			phi   = atan2(r/rho, v[DZ]/rho);
		}
	}
}

}
