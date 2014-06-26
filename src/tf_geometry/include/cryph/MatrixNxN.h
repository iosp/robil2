// MatrixNxN.h -- Subclass of MatrixMxN where M==N ===> square matrices
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef MATRIXNXN_H
#define MATRIXNXN_H

#include "MatrixMxN.h"

namespace cryph
{

class MatrixNxN : public MatrixMxN
{
public:
	// There is no default constructor. You must specify a size.
	MatrixNxN(int size); // builds an NxN identity matrix
	MatrixNxN(const MatrixNxN& rhs);
	// Following builds an NxN matrix from a^Ta. ("a^T" denotes "a-transpose".)
	// The matrix stored in 'a' is of size nRows x nCols. The resulting NxN
	// matrix will then have N=nCols.
	// NOTE: This is very different from the superclass constructor with the
	// same signature.
	MatrixNxN(const double** a, int nRows, int nCols);
	virtual ~MatrixNxN( );

	// --------- operators as methods
	// if the sizes are different in any of the following methods, the
	// operation is performed on the kxk submatrix common to both.
	MatrixNxN	operator=(const MatrixNxN& rhs);
	MatrixNxN	operator*=(const MatrixNxN& rhs);
	MatrixNxN	operator+=(const MatrixNxN& rhs);
	// just multiplies all elements of "this" matrix by f:
	MatrixNxN	operator*=(double f);

	// --------- operators
	// if the sizes are different in any of the following functions, the
	// operation is performed on the kxk submatrix common to both.
	MatrixNxN	operator*(const MatrixNxN& m2) const;
	MatrixNxN	operator+(const MatrixNxN& m2) const;
	MatrixNxN	operator-(const MatrixNxN& m2) const;
	// returns a matrix that is f*m
	friend MatrixNxN	operator*(double f, const MatrixNxN& m);

	// --------- General Methods

	double	determinant() const;

	int		getSize( ) const { return getNumRows(); }

	MatrixNxN	inverse() const;
	MatrixNxN	multiply(const MatrixNxN& rhs) const;

	// The following assumes that 'a' is an array of (at least) mNumCols and
	// that 'b' is an array of (at least) size mNumRows.
	// These are actually implemented in MatrixMxN, but we add these
	// declarations here to make the MxN versions visible.
	void		multiply(const double a[], double b[]) const // b = this*a
					{ MatrixMxN::multiply(a,b); }
	void		multiply(const AffPoint a[], AffPoint b[]) const // b = this*a
					{ MatrixMxN::multiply(a,b); }

 private:

	static void		lubksb(const MatrixNxN& A, int* indx, double* b);
	static void		ludcmp(MatrixNxN& A, int* indx, double& d);
};

}

#endif
