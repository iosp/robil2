// MatrixNxN -- General NxN square matrices with elements of type double
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <math.h>

#include <cryph/MatrixNxN.h>
using namespace std;

namespace cryph
{

// create an identity matrix of the given size

MatrixNxN::MatrixNxN(int size) : MatrixMxN(size,size)
{
}

MatrixNxN::MatrixNxN(const MatrixNxN& rhs) : MatrixMxN(rhs)
{
}

// Following builds an NxN matrix from a^Ta. ("a^T" denotes "a-transpose".)
// The matrix stored in 'a' is of size nRows x nCols. The resulting square NxN
// matrix will then be nCols x nCols.

MatrixNxN::MatrixNxN(const double** a, int nRows, int nCols) :
	MatrixMxN(nCols,nCols)
{
	if ( (nRows > 0) && (nCols > 0) )
	{
		for (int i=0 ; i<nCols ; i++) // indexes through rows of "a-transpose"
		{
			for (int j=0 ; j<nCols ; j++) // indexes across columns of "a"
			{
				double sum = 0.0;
				for (int k=0 ; k<nRows ; k++)
					    // a^T[i][k] * a[k][j]
					sum += a[k][i]   * a[k][j];
				mM[i][j] = sum;
			}
		}
	}
}

MatrixNxN::~MatrixNxN( )
{ 
}

// Method "determinant" derived from "Numerical Recipes in C",
// Press et al., 1988
double MatrixNxN::determinant() const
{
	// copy the original matrix
	MatrixNxN A(*this);
	int* indx = new int[mNumRows];
	double d;

	ludcmp(A,indx,d);
	for (int j=0 ; j<mNumCols ; j++)
		d *= A.mM[j][j];

	delete [] indx;
	return d;
}

// Method "inverse" derived from "Numerical Recipes in C", Press et al., 1988
MatrixNxN MatrixNxN::inverse() const
{
	// copy the original matrix

	MatrixNxN A(*this), Y(mNumRows);
	double* col = new double[mNumRows];
	int* indx = new int[mNumRows];
	double d;

	ludcmp(A,indx,d);
	for (int j=0 ; j<mNumRows ; j++)
	{
		int i = 0;
		for ( ; i<mNumRows ; i++)
			col[i] = 0.0;
		col[j] = 1.0;
		lubksb(A,indx,col);
		for (i=0 ; i<mNumRows ; i++)
			Y.mM[i][j] = col[i];
	}

	delete [] col;
	delete [] indx;

	return Y;
}

// Method "lubksb" derived from "Numerical Recipes in C", Press et al., 1988
void MatrixNxN::lubksb(const MatrixNxN& A, int* indx, double* b) // CLASS METHOD
{
	int i = 0, ii = -1;
	for ( ; i<A.mNumRows ; i++)
	{
		int ip = indx[i];
		double sum = b[ip];
		b[ip] = b[i];
		if (ii != -1)
			for (int j=ii ; j<=i-1 ; j++)
				sum -= A.mM[i][j]*b[j];
		else if (sum != 0.0)
			ii = i;
		b[i] = sum;
	}
	for (i=A.mNumRows-1 ; i>=0 ; i--)
	{
		double sum = b[i];
		for (int j=i+1 ; j<A.mNumRows ; j++)
			sum -= A.mM[i][j]*b[j];
		b[i] = sum / A.mM[i][i];
	}
}

// Method "ludcmp" derived from "Numerical Recipes in C", Press et al., 1988
void MatrixNxN::ludcmp(MatrixNxN& A, int* indx, double& d) // CLASS METHOD
{
	const double TINY = 1.0e-20;

	double* vv = new double[A.mNumRows];
	d = 1.0;
	int i = 0;
	double big, temp;
	for ( ; i<A.mNumRows ; i++)
	{
		big = 0.0;
		for (int j=0 ; j<A.mNumRows ; j++)
			if ((temp=fabs(A.mM[i][j])) > big)
				big = temp;
		if (big == 0.0)
			cerr << "MatrixNxN::ludcmp: Singular matrix.\n";
		else
			vv[i] = 1.0 / big;
	}

	double sum;
	for (int j=0 ; j<A.mNumRows ; j++)
	{
		for (i=0 ; i<j ; i++)
		{
			sum = A.mM[i][j];
			for (int k=0 ; k<i ; k++)
				sum -= A.mM[i][k]*A.mM[k][j];
			A.mM[i][j] = sum;
		}
		big = 0.0;
		double dum;
		int imax;
		for (i=j ; i<A.mNumRows ; i++)
		{
			sum = A.mM[i][j];
			for (int k=0 ; k<j ; k++)
				sum -= A.mM[i][k]*A.mM[k][j];
			A.mM[i][j] = sum;
			if ( (dum=vv[i]*fabs(sum)) >= big)
			{
				big = dum;
				imax = i;
			}
		}
		if (j != imax)
		{
			for (int k=0 ; k<A.mNumRows ; k++)
			{
				dum = A.mM[imax][k];
				A.mM[imax][k] = A.mM[j][k];
				A.mM[j][k] = dum;
			}
			d = -d;
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if (A.mM[j][j] == 0.0)
			A.mM[j][j] = TINY;
		if (j != A.mNumRows)
		{
			dum = 1.0 / A.mM[j][j];
			for (i=j+1 ; i<A.mNumRows ; i++)
				A.mM[i][j] *= dum;
		}
	}

	delete [] vv;
}

MatrixNxN MatrixNxN::multiply(const MatrixNxN& rhs) const
{
	int nR, nC;
	largestCommonSubmatrix(rhs,nR,nC);
	MatrixNxN retval(nR);

	for(int i = 0 ; i<nR; i++)
		for(int j =0 ; j<nR ; j++)
		{
			double sum = 0.0;
			for (int k=0 ; k<nR ; k++)
				sum += mM[i][k] * rhs.mM[k][j];
			retval.mM[i][j] = sum;
		}
	return retval;		
}

// if the sizes are different in any of the following methods, the
// operation is performed on the kxk submatrix common to both.

MatrixNxN MatrixNxN::operator=(const MatrixNxN& rhs)
{
	int nR, nC;
	largestCommonSubmatrix(rhs,nR,nC);
	for (int i=0 ; i<nR ; i++)
		for (int j=0 ; j<nR ; j++)
			mM[i][j] = rhs.mM[i][j];
	return *this;
}

MatrixNxN MatrixNxN::operator*=(const MatrixNxN& rhs)
{
	MatrixNxN mm = multiply(rhs);
	*this = mm;
	return mm;
}

MatrixNxN MatrixNxN::operator+=(const MatrixNxN& rhs)
{
	int nR, nC;
	largestCommonSubmatrix(rhs,nR,nC);
	for (int i=0 ; i<nR ; i++)
		for (int j=0 ; j<nR ; j++)
			mM[i][j] += rhs.mM[i][j];
	return *this;
}

// just multiplies all elements of "this" matrix by f:
MatrixNxN MatrixNxN::operator*=(double f)
{
	for (int i=0 ; i<mNumRows ; i++)
		for (int j=0 ; j<mNumCols ; j++)
			mM[i][j] *= f;
	return *this;
}

// if the sizes are different in any of the following functions, the
// operation is performed on the kxk submatrix common to both.

MatrixNxN MatrixNxN::operator*(const MatrixNxN& m2) const
{
	return this->multiply(m2);
}

MatrixNxN MatrixNxN::operator+(const MatrixNxN& m2) const
{
	int nR, nC;
	this->largestCommonSubmatrix(m2,nR,nC);
	MatrixNxN retval(nR);

	for (int i=0 ; i<nR ; i++)
		for (int j=0 ; j<nR ; j++)
			retval.mM[i][j] = this->mM[i][j] + m2.mM[i][j];
	return retval;
}

MatrixNxN MatrixNxN::operator-(const MatrixNxN& m2) const
{
	int nR, nC;
	this->largestCommonSubmatrix(m2,nR,nC);
	MatrixNxN retval(nR);

	for (int i=0 ; i<nR ; i++)
		for (int j=0 ; j<nR ; j++)
			retval.mM[i][j] = this->mM[i][j] - m2.mM[i][j];
	return retval;
}

// creates and returns a matrix M = f*m
MatrixNxN operator*(double f, const MatrixNxN& m)
{
	MatrixNxN ret(m.mNumRows);
	for (int i=0 ; i<m.mNumRows ; i++)
		for (int j=0 ; j<m.mNumRows ; j++)
			ret.mM[i][j] = f * m.mM[i][j];
	return ret;
}

}
