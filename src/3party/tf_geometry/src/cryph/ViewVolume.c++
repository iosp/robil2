// ViewVolume.c++ -- a generic 3D view volume
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <cryph/ViewVolume.h>
#include <cryph/AffPoint.h>
#include <cryph/AffVector.h>
#include <cryph/Inline.h>
#include <cryph/Tolerances.h>

using namespace std;

namespace cryph
{

AffPoint ViewVolume::center() const
{
	return AffPoint( 0.5*(xmin+xmax) , 0.5*(ymin+ymax), 0.5*(zmin+zmax) );
}

ViewVolume ViewVolume::operator+(const ViewVolume& rhs) const
{
	return ViewVolume(
		minimum(xmin,rhs.xmin), maximum(xmax,rhs.xmax),
		minimum(ymin,rhs.ymin), maximum(ymax,rhs.ymax),
		minimum(zmin,rhs.zmin), maximum(zmax,rhs.zmax)
			);
}

ViewVolume ViewVolume::operator+(const AffPoint& rhs) const
{
	return ViewVolume(
		minimum(xmin,rhs[X]), maximum(xmax,rhs[X]),
		minimum(ymin,rhs[Y]), maximum(ymax,rhs[Y]),
		minimum(zmin,rhs[Z]), maximum(zmax,rhs[Z])
			);
}

ViewVolume ViewVolume::operator+=(const ViewVolume& rhs)
{
	xmin = minimum(xmin,rhs.xmin);
	xmax = maximum(xmax,rhs.xmax);
	ymin = minimum(ymin,rhs.ymin);
	ymax = maximum(ymax,rhs.ymax);
	zmin = minimum(zmin,rhs.zmin);
	zmax = maximum(zmax,rhs.zmax);

	return *this;
}

ViewVolume ViewVolume::operator+=(const AffPoint& rhs)
{
	xmin = minimum(xmin,rhs[X]);
	xmax = maximum(xmax,rhs[X]);
	ymin = minimum(ymin,rhs[Y]);
	ymax = maximum(ymax,rhs[Y]);
	zmin = minimum(zmin,rhs[Z]);
	zmax = maximum(zmax,rhs[Z]);

	return *this;
}

void ViewVolume::bubbleSort(double N[], int n) // CLASS METHOD
{
	bool done = true;
	int  last = n-1;
	do
	{
		done = true;
		for (int i=0 ; i<last ; i++)
			if (N[i] > N[i+1])
			{
				cryph::swap2(N[i],N[i+1]);
				done = false;
			}
		last--;
	} while (!done);
}

bool ViewVolume::contains(const AffPoint& p) const
{
	double tol = 1.0e-7;

	if ( (p[X] >= xmin-tol) && (p[X] <= xmax+tol) )
		if ( (p[Y] >= ymin-tol) && (p[Y] <= ymax+tol) )
			if ( (p[Z] >= zmin-tol) && (p[Z] <= zmax+tol) )
				return true;
	return false;
}

double ViewVolume::diagonalLength() const
{
	return sqrt(diagonalSquaredLength());
}

double ViewVolume::diagonalSquaredLength() const
{
	double t = xmax - xmin;
	double sum = t*t;
	t = ymax - ymin;
	sum += (t*t);
	t = zmax - zmin;
	return (sum + t*t);
}

// if the line (Q,u) intersects the ViewVolume, compute the parameters
// where it enters and exits the ViewVolume and return true. Otherwise
// return false.
bool ViewVolume::intersect(const AffPoint& Q, const AffVector& u,
	double& p1, double& p2) const
{
	double	params[6];
	int		nHits = 0;

	AffVector uHat;
	if (u.normalizeToCopy(uHat) < BasicDistanceTol)
		// zero vector -- cannot proceed
		return false;

	if (fabs(uHat[DX]) > BasicUnitTol)
	{
		params[nHits++] = (xmin - Q[X]) / uHat[DX];
		params[nHits++] = (xmax - Q[X]) / uHat[DX];
	}
	else if ( (Q[X] < xmin) || (Q[X] > xmax) )
		return false;

	if (fabs(uHat[DY]) > BasicUnitTol)
	{
		params[nHits++] = (ymin - Q[Y]) / uHat[DY];
		params[nHits++] = (ymax - Q[Y]) / uHat[DY];
	}
	else if ( (Q[Y] < ymin) || (Q[Y] > ymax) )
		return false;

	if (fabs(uHat[DZ]) > BasicUnitTol)
	{
		params[nHits++] = (zmin - Q[Z]) / uHat[DZ];
		params[nHits++] = (zmax - Q[Z]) / uHat[DZ];
	}
	else if ( (Q[Z] < zmin) || (Q[Z] > zmax) )
		return false;

	bubbleSort(params,nHits);
	int i = nHits/2;
	p1 = params[i-1];
	p2 = params[i];
	return true;
}

void ViewVolume::makeCube()
{
	double sz = xmax - xmin;
	double d  = ymax - ymin;
	if (d > sz)
		sz = d;
	d = zmax - zmin;
	if (d > sz)
		sz = d;
	double hsz = 0.5 * sz;
	double mid = 0.5 * (xmin + xmax);
	xmin = mid - hsz;
	xmax = mid + hsz;
	mid = 0.5 * (ymin + ymax);
	ymin = mid - hsz;
	ymax = mid + hsz;
	mid = 0.5 * (zmin + zmax);
	zmin = mid - hsz;
	zmax = mid + hsz;
}

void ViewVolume::makeSquareInXY()
{
	double sz = xmax - xmin;
	double d  = ymax - ymin;
	if (d > sz)
		sz = d;
	double hsz = 0.5 * sz;
	double mid = 0.5 * (xmin + xmax);
	xmin = mid - hsz;
	xmax = mid + hsz;
	mid = 0.5 * (ymin + ymax);
	ymin = mid - hsz;
	ymax = mid + hsz;
}

double ViewVolume::maxSideLength(int* which) const
{
	double max = xmax - xmin;
	int w = 0;
	double t = ymax - ymin;
	if (t > max)
	{
		max = t;
		w = 1;
	}
	t = zmax - zmin;
	if (t > max)
	{
		max = t;
		w = 2;
	}
	if (which != NULL) *which = w;
	return max;
}

double ViewVolume::minSideLength(int* which) const
{
	double min = xmax - xmin;
	int w = 0;
	double t = ymax - ymin;
	if (t < min)
	{
		min = t;
		w = 1;
	}
	t = zmax - zmin;
	if (t < min)
	{
		min = t;
		w = 2;
	}
	if (which != NULL) *which = w;
	return min;
}

bool ViewVolume::overlaps(const ViewVolume& other) const
{
	if (xmin > other.xmax)
		return false;
	if (xmax < other.xmin)
		return false;
	if (ymin > other.ymax)
		return false;
	if (ymax < other.ymin)
		return false;
	if (zmin > other.zmax)
		return false;
	if (zmax < other.zmin)
		return false;
	return true;
}

} // end namespace cryph

istream& operator>>(istream& is, cryph::ViewVolume& v)
{
	is >> v.xmin >> v.xmax;
	is >> v.ymin >> v.ymax;
	is >> v.zmin >> v.zmax;
	return is;
}

ostream& operator<<(ostream& os, const cryph::ViewVolume& v)
{
	os << v.xmin << ' ' << v.xmax << ' ';
	os << v.ymin << ' ' << v.ymax << ' ';
	os << v.zmin << ' ' << v.zmax;
	return os;
}
