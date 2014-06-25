// Inline.h  -- Definition of common low-level inline functions
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef INLINE_H
#define INLINE_H

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>

namespace cryph
{

inline std::string closeStr(char openStr)
{
	if (openStr == '(')
		return ")";
	if (openStr == '[')
		return "]";
	if (openStr == '{')
		return "}";
	if (openStr == '<')
		return ">";
	if (openStr == ' ')
		return " ";
	return "???";
}

inline double degreesToRadians(double angleInDegrees)
{
	return angleInDegrees*M_PI/180.0;
}

inline bool equalScalars(double s1, double s2, double tol)
{
	return (fabs(s1-s2) < tol);
}

inline int maximum(int s1, int s2)
{
	return (s1 >= s2) ? s1 : s2;
}

inline double maximum(double s1, double s2)
{
	return (s1 >= s2) ? s1 : s2;
}

inline int minimum(int s1, int s2)
{
	return (s1 >= s2) ? s2 : s1;
}

inline double minimum(double s1, double s2)
{
	return (s1 >= s2) ? s2 : s1;
}

inline double radiansToDegrees(double angleInRadians)
{
	return angleInRadians*180.0/M_PI;
}

inline double random0To1_next()
{
	return drand48();
}

inline void random0To1_seed(long seedVal)
{
	srand48(seedVal);
}

inline int roundR2I(double val)
{
	return (int) (val + 0.5);
}

inline void skipNonblankChars(std::istream& is, int nNonBlankChars)
{
	char ch;
	for (int i=0 ; i<nNonBlankChars ; i++)
		is >> ch;
}

inline double square(double s)
{
	return s * s;
}

// Templates

template <typename T> inline void swap2(T& t1, T& t2)
{
	T temp = t1;
	t1 = t2;
	t2 = temp;
}

}
 
#endif
