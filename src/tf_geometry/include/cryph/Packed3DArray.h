// Packed3DArray.h -- A Template class that can be used to build a 3D array
//                    of objects of type T.
//                    See also "Packed2DArray" and "ImageReader".
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

//  Documentation on use:
//    http://people.eecs.ku.edu/~miller/Courses/ToolDoc/Packed3DArray.html
//
//
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
//      WARNING: Current implementation assumes 'T' is an integral type, hence
//               the implementation of the I/O operators below employ 'int'.

#ifndef PACKED3DARRAY_H
#define PACKED3DARRAY_H

#include <iostream>

namespace cryph
{

template <typename T>
class Packed3DArray
{
public:
	// --------- basic constructors
	Packed3DArray(int dim1=2, int dim2=2, int dim3=3);
	Packed3DArray(const Packed3DArray<T>& t3da);

	// --------- destructor
	virtual ~Packed3DArray();

	// ---------- General Methods
	const T*	getData() const { return mData; }
	T			getDataElement(int i1, int i2, int i3) const;
	T*			getDataElementLoc(int i1, int i2, int i3) const;
	int			getDim1() const { return mDim1; }
	int			getDim2() const { return mDim2; }
	int			getDim3() const { return mDim3; }
	T*			getModifiableData() { return mData; }
	int			getTotalNumberElements() const;
	void		setDataElement(int i1, int i2, int i3, const T& elem);

	// --------- Class methods
	static void	setErrorReporting(bool r);
	static void	setOutOfBoundsValue(T defaultValue);

private:

	int		getOffset(const char* routine, int i1, int i2, int i3) const;

	T*		mData;
	int		mDim1;
	int		mDim2;
	int		mDim3;

	static	bool	sReportErrors;
	static	T		sOutOfBoundsValue;
};

template <typename T>
bool	Packed3DArray<T>::sReportErrors = true;

template <typename T>
T		Packed3DArray<T>::sOutOfBoundsValue;

template <typename T>
Packed3DArray<T>::Packed3DArray(int dim1, int dim2, int dim3) :
		mData(NULL), mDim1(dim1), mDim2(dim2), mDim3(dim3)
{
	if ( (dim1 < 1) || (dim2 < 1) || (dim3 < 1) )
	{
		mDim1 = 0; mDim2 = 0; mDim3 = 0;
		if (Packed3DArray<T>::sReportErrors)
			std::cerr << "Invalid dimensions in constructor: ("
			     << dim1 << ", " << dim2 << ", " << dim3 << ')' << std::endl;
	}
	else
		mData = new T[dim1*dim2*dim3];
}

template <typename T>
Packed3DArray<T>::Packed3DArray(const Packed3DArray<T>& t3da) :
		mData(NULL), mDim1(t3da.mDim1), mDim2(t3da.mDim2), mDim3(t3da.mDim3)
{
	int		size = mDim1 * mDim2 * mDim3;
	mData = new T[size];
	for (int i=0 ; i<size ; i++)
		mData[i] = t3da.mData[i];
}

template <typename T>
Packed3DArray<T>::~Packed3DArray()
{
	if (mData != NULL)
	{
		delete [] mData;
		mData = NULL;
		mDim1 = 0;
		mDim2 = 0;
		mDim3 = 0;
	}
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Packed3DArray<T>& t3da)
{
	int size = t3da.getTotalNumberElements();
	const T* Tarr = t3da.getData();
	for (int i=0 ; i<size ; i++)
	{
		// WARNING: Assuming integral type for now, hence cast to integer
		//          first. For example, if "typename T" is GLubyte, this ensures
		//          that full 8-bit ints are written as opposed to general
		//          ASCII chars.
		os << (int)Tarr[i];
		if ((i % 20) == 0)
			os << '\n';
		else
			os << ' ';
	}
	os << '\n';
	return os;
}

template <typename T>
std::istream& operator>>(std::istream& is, Packed3DArray<T>& t3da)
{
	int size = t3da.getTotalNumberElements();
	int temp = 0;
	T* Tarr = t3da.getModifiableData();
	for (int i=0 ; i<size ; i++)
	{
		// WARNING: See 'WARNING' in "operator<<" above.
		is >> temp;
		Tarr[i] = (T)temp;
	}
	return is;
}

template <typename T>
T Packed3DArray<T>::getDataElement(int i1, int i2, int i3) const
{
	int loc = getOffset("getDataElement",i1,i2,i3);
	if (loc < 0)
	{
		return Packed3DArray<T>::sOutOfBoundsValue;
	}
	return mData[loc];
}

template <typename T>
T* Packed3DArray<T>::getDataElementLoc(int i1, int i2, int i3) const
{
	int loc = getOffset("getDataElementLoc",i1,i2,i3);
	if (loc < 0)
		return NULL;
	return &mData[loc];
}

template <typename T>
int Packed3DArray<T>::getOffset(const char* routine, int i1, int i2, int i3)
	const
{
	if ( (i1 < 0) || (i1 >= mDim1) ||
	     (i2 < 0) || (i2 >= mDim2) ||
	     (i3 < 0) || (i3 >= mDim3) )
	{
		if (Packed3DArray<T>::sReportErrors)
			std::cerr << routine << ": Invalid data element reference: ("
			     << i1 << ", " << i2 << ", " << i3 << ')' << std::endl;
		return -1;
	}
	return (i1 * mDim2 * mDim3) + (i2 * mDim3) + i3;
}

template <typename T>
int Packed3DArray<T>::getTotalNumberElements() const
{
	return mDim1 * mDim2 * mDim3;
}

template <typename T>
void Packed3DArray<T>::setDataElement(int i1, int i2, int i3, const T& elem)
{
	int loc = getOffset("setDataElement",i1,i2,i3);
	if (loc >= 0)
		mData[loc] = elem;
}

template <typename T>
void Packed3DArray<T>::setErrorReporting(bool r)
{
	Packed3DArray<T>::sReportErrors = r;
}

template <typename T>
void Packed3DArray<T>::setOutOfBoundsValue(T defaultValue)
{
	Packed3DArray<T>::sOutOfBoundsValue = defaultValue;
}

}

#endif
