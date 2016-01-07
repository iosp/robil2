// MatrixMxN.h -- General MxN matrices with elements of type double
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef MATRIXMXN_H
#define MATRIXMXN_H

#include <iostream>

namespace cryph
{
class AffPoint;

class MatrixMxN
{
public:
	// There is no default constructor. You must specify a size.
	MatrixMxN(int nRows, int nCols); // builds an MxN identity matrix
	MatrixMxN(const MatrixMxN& rhs);
	MatrixMxN(const double** a, int nRows, int nCols);
	virtual ~MatrixMxN( );

	// --------- operators as methods
	// if the sizes are incompatible in any of the following methods, the
	// operation is performed on the largest compatible submatrices
	MatrixMxN	operator=(const MatrixMxN& rhs);
	MatrixMxN	operator*=(const MatrixMxN& rhs);
	MatrixMxN	operator+=(const MatrixMxN& rhs);
	// just multiplies all elements of "this" matrix by f:
	MatrixMxN	operator*=(double f);

	// --------- operators
	// if the sizes are incompatible in any of the following functions, the
	// operation is performed on the largest compatible submatrices
	MatrixMxN	operator*(const MatrixMxN& m2) const;
	MatrixMxN	operator+(const MatrixMxN& m2) const;
	MatrixMxN	operator-(const MatrixMxN& m2) const;
	// returns a matrix that is f*m
	friend MatrixMxN	operator*(double f, const MatrixMxN& m);
	friend std::ostream&	operator<<(std::ostream& os, const MatrixMxN& m);
	friend std::istream&	operator>>(std::istream& is,       MatrixMxN& m);

	// --------- General Methods

	double	elementAt(int i, int j) const;

	// getCol and getRow return a dynamically allocated array of doubles
	// of the appropriate length. The caller is responsible for deleting
	// the array.
	double*	getCol(int j) const; 
	int		getNumCols( ) const { return mNumCols; }
	int		getNumRows( ) const { return mNumRows; }
	double*	getRow(int i) const;

	void	makeIdentity();
	// if the sizes are incompatible in the following, the operation
	// is performed on the largest compatible submatrices
	MatrixMxN	multiply(const MatrixMxN& rhs) const;

	// The following assumes that 'a' is an array of (at least) mNumCols and
	// that 'b' is an array of (at least) size mNumRows.
	void		multiply(const double a[], double b[]) const; // b = this*a
	void		multiply(const AffPoint a[], AffPoint b[]) const; // b = this*a

	void	setCol(int j, const double rhs[]);
	void	setElementAt(int i, int j, double val);
	void	setRow(int i, const double rhs[]);

	MatrixMxN	subMatrix(int skipRow, int skipCol) const;

protected:

	int		mNumRows, mNumCols;
	double	**mM;

	void	largestCommonSubmatrix(const MatrixMxN& m, int& nRows, int& nCols)
				const
				{ if (mNumRows < m.mNumRows) nRows = mNumRows;
				  else nRows = m.mNumRows;
				  if (mNumCols < m.mNumCols) nCols = mNumCols;
				  else nCols = m.mNumCols;
				}
	int		determineMultiplicationRegion(const MatrixMxN& rhs) const
				{
					if (mNumCols < rhs.mNumRows)
						return mNumCols;
					return rhs.mNumRows;
				}
private:

	static double**	copy(const double** from, int nR, int nC);
};

}

#endif
