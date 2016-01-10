// ViewVolume.h -- a generic 3D view volume
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef VIEWVOLUME_H
#define VIEWVOLUME_H

#include <fstream>

#include "AffPoint.h"

namespace cryph
{
class AffVector;

struct ViewVolume
{
	ViewVolume() : xmin(-1.0), xmax(1.0), ymin(-1.0), ymax(1.0),
                   zmin(-1.0), zmax(1.0) {}
	ViewVolume(double a, double b, double c,
		double d, double e, double f) :
				xmin(a), xmax(b), ymin(c), ymax(d),
				zmin(e), zmax(f) {}
	ViewVolume(const AffPoint& p) :
		xmin(p[X]), xmax(p[X]), ymin(p[Y]), ymax(p[Y]), zmin(p[Z]), zmax(p[Z])
		{}
	double	xmin;
	double	xmax;
	double	ymin;
	double	ymax;
	double	zmin;
	double	zmax;

	AffPoint		center() const;
	ViewVolume operator+(const ViewVolume& rhs) const;
	ViewVolume operator+(const AffPoint& rhs) const;
	ViewVolume operator+=(const ViewVolume& rhs);
	ViewVolume operator+=(const AffPoint& rhs);
	static void bubbleSort(double N[], int n);
	bool contains(const AffPoint& p) const;
	double diagonalLength() const;
	double diagonalSquaredLength() const;

	// if the line (Q,u) intersects the ViewVolume, compute the parameters
	// where it enters and exits the ViewVolume and return true. Otherwise
	// return false.
	bool intersect(const AffPoint& Q, const AffVector& u,
		double& p1, double& p2) const;
	void makeCube();
	void makeSquareInXY();
	double maxSideLength(int* which=NULL) const;
	double minSideLength(int* which=NULL) const;
	bool overlaps(const ViewVolume& other) const;
};

}

std::istream& operator>>(std::istream& is, cryph::ViewVolume& v);
std::ostream& operator<<(std::ostream& os, const cryph::ViewVolume& v);

#endif
