// Basic.h
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef BASIC_H
#define BASIC_H

#include <iostream>
#include <fstream>
#include <string>

#include <math.h>
#include <stdlib.h>
 
namespace cryph
{
class AffPoint;
class AffVector;

typedef double real; // for compatibility with old code...

// Function prototypes

// 1. Dynamic Allocation
// Non-template versions of 2D/3D array allocation/deallocation do not
// require an initial value to be specified.

void		delete2DArrayD(double**& a);
void		delete2DArrayF(float**& a);
void		delete3DArrayD(double***& a);
void		delete3DArrayF(float***& a);
double**	new2DArrayD(int rows, int cols);
float**		new2DArrayF(int rows, int cols);
double***	new3DArrayD(int dim1, int dim2, int dim3);
float***	new3DArrayF(int dim1, int dim2, int dim3);
void		outputElement(std::ostream& os, double elem);

// 2. Other basic stuff

char*		copyString(const char* str); // returns a 'new'-ly allocated copy
int			intLinePlane(
				const AffPoint& linePnt,  const AffVector& lineDir,
				const AffPoint& plnPoint, const AffVector& plnNormal,
				AffPoint& intPoint);
std::string lowerCaseString(const std::string& str);
bool		pointOnPlane(AffPoint Q, AffPoint plnPoint, AffVector plnNormal);

// compute (a,b) so that a value, v, in the "from" range is
// mapped to a corresponding value in the "to" range as:
//   vTo = a*vFrom + b
void linearMap(double fromMin, double fromMax, double toMin, double toMax,
                        double& a, double& b);

// 3. Template versions of Dynamic allocation

// Template versions of 2D/3D array allocation/deallocation. An initial
// value must be specified, partly because C++ cannot recognize the correct
// template reference unless an item based on type "T" appears in the formal
// parameter list.

template <typename T>
void delete2DArray(T**& a)
{
	if (a == NULL)
		return;

	int i = 0;
	while (a[i] != NULL)
	{
		delete [] a[i];
		a[i++] = NULL;
	}
	delete [] a;
	a = NULL;
}

template <typename T>
void delete3DArray(T***& a)
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
			a[i][j++] = NULL;
		}
		delete [] a[i];
		a[i++] = NULL;
	}
	delete [] a;
	a = NULL;
}

template <typename T>
T**			new2DArray(int nRows, int nCols, const T& initialValue)
{
	T** a = new T*[nRows+1]; // one extra to use as a "sentinel" for "delete"
	for (int i=0 ; i<nRows ; i++)
	{
		a[i] = new T[nCols];
		for (int j=0 ; j<nCols ; j++)
			a[i][j] = initialValue;
	}
	a[nRows] = NULL;
	return a;
}

template <typename T>
T*** new3DArray(int dim1, int dim2, int dim3, const T& initialValue)
{
	T*** a = new T**[dim1+1]; // add a sentinel for "delete" to find
	for (int i=0 ; i<dim1 ; i++)
	{
		a[i] = new T*[dim2+1]; // add a sentinel for "delete" to find
		for (int j=0 ; j<dim2 ; j++)
		{
			a[i][j] = new T[dim3];
			for (int k=0 ; k<dim3 ; k++)
				a[i][j][k] = initialValue;
		}
		a[i][dim2] = NULL;
	}
	a[dim1] = NULL;
	return a;
}

} // end namespace cryph
 
#endif
