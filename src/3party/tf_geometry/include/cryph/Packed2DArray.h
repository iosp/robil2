// Packed2DArray.h -- A Template class that can be used to build a 2D array
//                    of objects of type T.
//                    See also "Packed3DArray" and "ImageReader".
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

//  Documentation on use:
//    http://people.eecs.ku.edu/~miller/Courses/ToolDoc/Packed2DArray.html
// To use this template class, the type used for "T" must be EITHER
//    * a primitive type
// OR
//    * a class that has the following public methods:
//      --> a default constructor (to support error condition handling in
//          "getDataElement")
//      --> a copy constructor (also for "getDataElement")
//      --> "operator="
//      and for which the standard I/O operators are defined:
//      --> "operator>>" and "operator<<"

#ifndef PACKED2DARRAY_H
#define PACKED2DARRAY_H

#include <iostream>

namespace cryph
{

template <typename T>
class Packed2DArray
{
public:
	// --------- basic constructors
	Packed2DArray(int dim1=2, int dim2=2);
	Packed2DArray(const Packed2DArray<T>& t2da);

	// --------- destructor
	virtual ~Packed2DArray();

	// ---------- General Methods
	const T*	getData() const;
	T			getDataElement(int i1, int i2) const;
	const T*	getDataElementLoc(int i1, int i2) const;
	int			getDim1() const;
	int			getDim2() const;
	T*			getModifiableData();
	int			getTotalNumberElements() const;
	void		setDataElement(int i1, int i2, const T& elem);

	// --------- Class methods
	static void	setErrorReporting(bool r);
	static void	setOutOfBoundsValue(T defaultValue);

private:

	int		getOffset(const char* routine, int i1, int i2) const;

	T*		mData;
	int		mDim1;
	int		mDim2;

	static	bool	sReportErrors;
	static	T		sOutOfBoundsValue;
};

template <typename T>
bool	Packed2DArray<T>::sReportErrors = true;

template <typename T>
T		Packed2DArray<T>::sOutOfBoundsValue;

template <typename T>
Packed2DArray<T>::Packed2DArray(int dim1, int dim2) :
		mData(NULL), mDim1(dim1), mDim2(dim2)
{
	if ( (dim1 < 1) || (dim2 < 1) )
	{
		mDim1 = 0; mDim2 = 0;
		if (Packed2DArray<T>::sReportErrors)
			std::cerr << "Invalid dimensions in constructor: ("
			     << dim1 << ", " << dim2 << ')' << std::endl;
	}
	else
		mData = new T[dim1*dim2];
}

template <typename T>
Packed2DArray<T>::Packed2DArray(const Packed2DArray<T>& t2da) :
		mData(NULL), mDim1(t2da.mDim1), mDim2(t2da.mDim2)
{
	int		size = mDim1 * mDim2;
	mData = new T[size];
	for (int i=0 ; i<size ; i++)
		mData[i] = t2da.mData[i];
}

template <typename T>
Packed2DArray<T>::~Packed2DArray()
{
	if (mData != NULL)
	{
		delete [] mData;
		mData = NULL;
		mDim1 = 0;
		mDim2 = 0;
	}
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Packed2DArray<T>& t2da)
{
	const T* d = t2da.getData();
	int size = t2da.getTotalNumberElements();
	for (int i=0 ; i<size ; i++)
		os << d[i] << ' ';
	return os;
}

template <typename T>
std::istream& operator>>(std::istream& is, Packed2DArray<T>& t2da)
{
	int size = t2da.getTotalNumberElements();
	T* d = t2da.getModifiableData();
	for (int i=0 ; i<size ; i++)
		is >> d[i];
	return is;
}

template <typename T>
const T* Packed2DArray<T>::getData() const
{
	return mData;
}

template <typename T>
T Packed2DArray<T>::getDataElement(int i1, int i2) const
{
	int loc = getOffset("getDataElement",i1,i2);
	if (loc < 0)
	{
		return Packed2DArray<T>::sOutOfBoundsValue;
	}
	return mData[loc];
}

template <typename T>
const T* Packed2DArray<T>::getDataElementLoc(int i1, int i2) const
{
	int loc = getOffset("getDataElementLoc",i1,i2);
	if (loc < 0)
		return NULL;
	return &mData[loc];
}

template <typename T>
int Packed2DArray<T>::getDim1() const
{
	return mDim1;
}

template <typename T>
int Packed2DArray<T>::getDim2() const
{
	return mDim2;
}

template <typename T>
T* Packed2DArray<T>::getModifiableData()
{
	return mData;
}

template <typename T>
int Packed2DArray<T>::getOffset(const char* routine, int i1, int i2) const
{
	if ( (i1 < 0) || (i1 >= mDim1) ||
	     (i2 < 0) || (i2 >= mDim2) )
	{
		if (Packed2DArray<T>::sReportErrors)
			std::cerr << routine << ": Invalid data element reference: ("
			     << i1 << ", " << i2 << ')' << std::endl;
		return -1;
	}
	return (i1 * mDim2) + i2;
}

template <typename T>
int Packed2DArray<T>::getTotalNumberElements() const
{
	return mDim1 * mDim2;
}

template <typename T>
void Packed2DArray<T>::setDataElement(int i1, int i2, const T& elem)
{
	int loc = getOffset("setDataElement",i1,i2);
	if (loc >= 0)
		mData[loc] = elem;
}

template <typename T>
void Packed2DArray<T>::setErrorReporting(bool r)
{
	Packed2DArray<T>::sReportErrors = r;
}

template <typename T>
void Packed2DArray<T>::setOutOfBoundsValue(T defaultValue)
{
	Packed2DArray<T>::sOutOfBoundsValue = defaultValue;
}

}

#endif
