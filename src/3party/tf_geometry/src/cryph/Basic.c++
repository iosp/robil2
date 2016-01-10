// Basic.c++ -- support for various basic low level operations
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <string.h>

#include <cryph/Basic.h>
#include <cryph/Inline.h>
#include <cryph/AffPoint.h>
#include <cryph/AffVector.h>
#include <cryph/Tolerances.h>
using namespace std;

namespace cryph
{

char* copyString(const char* str)
{
	char* c = new char[strlen(str)+1];
	strcpy(c,str);
	return c;
}

void delete2DArrayD(double**& a)
{
	if (a == NULL)
		return;

	int i = 0;
	while (a[i] != NULL)
	{
		delete [] a[i];
		i++;
	}
	delete [] a;
	a = NULL;
}

void delete2DArrayF(float**& a)
{
	if (a == NULL)
		return;

	int i = 0;
	while (a[i] != NULL)
	{
		delete [] a[i];
		i++;
	}
	delete [] a;
	a = NULL;
}

void delete3DArrayD(double***& a)
{
	if (a == NULL)
		return;

	int i = 0;
	while (a[i] != NULL)
	{
		int j = 0;
		while (a[i][j] != NULL)
		{
			delete [] a[i][j];
			j++;
		}
		delete [] a[i];
		i++;
	}
	delete [] a;
	a = NULL;
}

void delete3DArrayF(float***& a)
{
	if (a == NULL)
		return;

	int i = 0;
	while (a[i] != NULL)
	{
		int j = 0;
		while (a[i][j] != NULL)
		{
			delete [] a[i][j];
			j++;
		}
		delete [] a[i];
		i++;
	}
	delete [] a;
	a = NULL;
}

int intLinePlane(
		const AffPoint& linePnt,  const AffVector& lineDir,
		const AffPoint& plnPoint, const AffVector& plnNormal,
		AffPoint& intPoint)
{
	// Return value "-1" ==> line parallel to plane
	// Return value " 0" ==> line contained in plane
	// Return value "+1" ==> single point fo intersection returned in "intPoint"

	AffVector	nHat, uHat;
	plnNormal.normalizeToCopy(nHat);
	lineDir.normalizeToCopy(uHat);
	double uDOTn = uHat.dot(nHat);
	if (fabs(uDOTn) < BasicUnitTol)
		// line direction is parallel to plane (or zero vectors given)
		if (pointOnPlane(linePnt,plnPoint,plnNormal))
			return 0;
		else
			return -1;

	double numerator = nHat.dot(plnPoint - linePnt);
	double distance = numerator / uDOTn;

	intPoint = linePnt + distance*uHat;
	return 1;
}

// compute (a,b) so that a value, v, in the "from" range is
// mapped to a corresponding value in the "to" range as:
//   vTo = a*vFrom + b
void linearMap(double fromMin, double fromMax, double toMin, double toMax,
					double& a, double& b)
{
	a = (toMax - toMin) / (fromMax - fromMin);
	b  = toMin - a*fromMin;
}
	
std::string lowerCaseString(const std::string& str)
{
	int n = str.length();
	char* lower = new char[n+1];
	const char* str_c = str.c_str();
	for (int i=0 ; i<n ; i++)
		lower[i] = tolower(str_c[i]);
	lower[n] = '\0';
	std::string out(lower);
	delete [] lower;
	return out;
}

double** new2DArrayD(int rows, int cols)
{
	double** a = new double*[rows+1]; // add a sentinel for "delete" to find
	for (int i=0 ; i<rows ; i++)
		a[i] = new double[cols];
	a[rows] = NULL;
	return a;
}

float**  new2DArrayF(int rows, int cols)
{
	float** a = new float*[rows+1]; // add a sentinel for "delete" to find
	for (int i=0 ; i<rows ; i++)
		a[i] = new float[cols];
	a[rows] = NULL;
	return a;
}

double*** new3DArrayD(int dim1, int dim2, int dim3)
{
	double*** a = new double**[dim1+1]; // add a sentinel for "delete" to find
	for (int i=0 ; i<dim1 ; i++)
	{
		a[i] = new double*[dim2+1]; // add a sentinel for "delete" to find
		for (int j=0 ; j<dim2 ; j++)
			a[i][j] = new double[dim3];
		a[i][dim2] = NULL;
	}
	a[dim1] = NULL;
	return a;
}

float***  new3DArrayF(int dim1, int dim2, int dim3)
{
	float*** a = new float**[dim1+1]; // add a sentinel for "delete" to find
	for (int i=0 ; i<dim1 ; i++)
	{
		a[i] = new float*[dim2+1]; // add a sentinel for "delete" to find
		for (int j=0 ; j<dim2 ; j++)
			a[i][j] = new float[dim3];
		a[i][dim2] = NULL;
	}
	a[dim1] = NULL;
	return a;
}

void outputElement(ostream& os, double elem)
{
	const double reallySmallNumber = 1.0e-10;
	const double zero = 0.0;

	if (fabs(elem) < reallySmallNumber)
		os << zero;
	else
		os << elem;
}

bool pointOnPlane(AffPoint Q, AffPoint plnPoint, AffVector plnNormal)
{
	AffVector nHat;
	if (plnNormal.normalizeToCopy(nHat) < BasicDistanceTol)
		// a zero vector was given
		return false;

	return ( fabs(nHat.dot(Q-plnPoint)) < BasicDistanceTol );
}

}
