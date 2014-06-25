// MatrixMxN -- General MxN matrices with elements of type double
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <math.h>

#include <cryph/MatrixMxN.h>
#include <cryph/AffPoint.h>
#include <cryph/Basic.h>

using namespace std;

namespace cryph
{

// create an identity matrix of the given size

MatrixMxN::MatrixMxN(int nRows, int nCols) : mNumRows(nRows), mNumCols(nCols)
{
	if ((mNumRows > 0) && (mNumCols > 0))
	{
		mM = new2DArrayD(mNumRows,mNumCols);
		makeIdentity();
	}
	else
	{
		mNumRows = mNumCols = 0; mM = NULL;
	}
}

MatrixMxN::MatrixMxN(const MatrixMxN& rhs) :
	mNumRows(rhs.mNumRows), mNumCols(rhs.mNumCols)
{
	mM = copy(const_cast<const double**>(rhs.mM) , mNumRows, mNumCols);
}

MatrixMxN::MatrixMxN(const double** a, int nRows, int nCols) :
	mNumRows(nRows), mNumCols(nCols)
{
	if ( (mNumRows > 0) && (mNumCols > 0) )
		mM = copy(a, mNumRows, mNumCols);
	else
	{
		mNumRows = mNumCols = 0; mM = NULL;
	}
}

MatrixMxN::~MatrixMxN( )
{ 
	delete2DArrayD(mM);
}

double** MatrixMxN::copy(const double** from, int nR, int nC) // CLASS METHOD
{
	double** M = new2DArrayD(nR,nC);
	for(int i = 0 ; i < nR ; i++)
		for(int j = 0 ; j < nC ; j++)
			M[i][j] = from[i][j];
	return M;
}

double MatrixMxN::elementAt(int i, int j) const
{
	if ((i >= 0) && (i < mNumRows) && (j >= 0) && (j < mNumCols))
		return mM[i][j];
	return 0.0;
}

double* MatrixMxN::getCol(int j) const
{
	double* retval = new double[mNumRows];
	for(int i = 0; i < mNumRows; i++)
		retval[i] = mM[i][j];
	return retval;
}

double* MatrixMxN::getRow(int i) const
{
	double* retval = new double[mNumCols];
	for(int j = 0; j < mNumCols; j++)
		retval[j] = mM[i][j];
	return retval;
}

void MatrixMxN::makeIdentity()
{
	for (int i=0 ; i<mNumRows ; i++)
	{
		for (int j=0 ; j<mNumCols ; j++)
			if (i == j)
				mM[i][i] = 1.0;
			else
				mM[i][j] = 0.0;
	}
}

MatrixMxN MatrixMxN::multiply(const MatrixMxN& rhs) const
{
	MatrixMxN retval(mNumRows,rhs.mNumCols);

	int kLimit = determineMultiplicationRegion(rhs);
	for(int i = 0 ; i<mNumRows; i++)
		for(int j =0 ; j<rhs.mNumCols ; j++)
		{
			double sum = 0.0;
			for (int k=0 ; k<kLimit ; k++)
				sum += mM[i][k] * rhs.mM[k][j];
			retval.mM[i][j] = sum;
		}
	return retval;		
}

// The following assumes that 'a' is an array of (at least) mNumCols and that
// 'b' is an array of (at least) size mNumRows.

void MatrixMxN::multiply(const double a[], double b[]) const
{
	for(int i=0 ; i<mNumRows ; i++)
	{
		double sum = 0.0;
		for (int j=0 ; j<mNumCols ; j++)
			sum += mM[i][j] * a[j];
		b[i] = sum;
	}
}

// The following method assumes that 'a' is at least of size mNumCols and
// that 'b' is at least of size mNumRows.

void MatrixMxN::multiply(const AffPoint a[], AffPoint b[]) const
{
	// temporary storage for coordinates of input points (ip) and
	// converted points (cp)
	double* ipCoord = new double[mNumCols];  // used once each for x, y, and z
	double** cpCoord = new double*[3];
	int c = 0;
	for ( ; c<3 ; c++)
		cpCoord[c] = new double[mNumRows];
	int k = 0;
	for (c=0 ; c<3 ; c++) // for each coord x, y, z:
	{
		for (k=0 ; k<mNumCols ; k++)
			ipCoord[k] = a[k][c];
		multiply(ipCoord,cpCoord[c]);
	}
	for (k=0 ; k<mNumRows ; k++)
		b[k] = AffPoint(cpCoord[0][k],cpCoord[1][k],cpCoord[2][k]);

	// clean up the mess...
	delete [] ipCoord;
	for (c=0 ; c<3 ; c++)
		delete [] cpCoord[c];
	delete [] cpCoord;
}

// if the sizes are different in any of the following methods, the
// operation is performed on the largest compatible submatrix.

MatrixMxN MatrixMxN::operator=(const MatrixMxN& rhs)
{
	int nR, nC;
	largestCommonSubmatrix(rhs, nR, nC);
	for (int i=0 ; i<nR ; i++)
		for (int j=0 ; j<nC ; j++)
			mM[i][j] = rhs.mM[i][j];
	return *this;
}

MatrixMxN MatrixMxN::operator*=(const MatrixMxN& rhs)
{
	MatrixMxN mm = multiply(rhs);
	*this = mm;
	return mm;
}

MatrixMxN MatrixMxN::operator+=(const MatrixMxN& rhs)
{
	int nR, nC;
	largestCommonSubmatrix(rhs, nR, nC);
	for (int i=0 ; i<nR ; i++)
		for (int j=0 ; j<nC ; j++)
			mM[i][j] += rhs.mM[i][j];
	return *this;
}

// just multiplies all elements of "this" matrix by f:
MatrixMxN MatrixMxN::operator*=(double f)
{
	for (int i=0 ; i<mNumRows ; i++)
		for (int j=0 ; j<mNumCols ; j++)
			mM[i][j] *= f;
	return *this;
}

// if the sizes are different in any of the following functions, the
// operation is performed on the kxk submatrix common to both.

MatrixMxN MatrixMxN::operator*(const MatrixMxN& m2) const
{
	return this->multiply(m2);
}

MatrixMxN MatrixMxN::operator+(const MatrixMxN& m2) const
{
	int nR, nC;
	this->largestCommonSubmatrix(m2, nR, nC);
	MatrixMxN retval(nR,nC);

	for (int i=0 ; i<nR ; i++)
		for (int j=0 ; j<nC ; j++)
			retval.mM[i][j] = this->mM[i][j] + m2.mM[i][j];
	return retval;
}

MatrixMxN MatrixMxN::operator-(const MatrixMxN& m2) const
{
	int nR, nC;
	this->largestCommonSubmatrix(m2, nR, nC);
	MatrixMxN retval(nR,nC);

	for (int i=0 ; i<nR ; i++)
		for (int j=0 ; j<nC ; j++)
			retval.mM[i][j] = this->mM[i][j] - m2.mM[i][j];
	return retval;
}

// creates and returns a matrix M = f*m
MatrixMxN operator*(double f, const MatrixMxN& m)
{
	MatrixMxN ret(m.mNumRows,m.mNumCols);
	for (int i=0 ; i<m.mNumRows ; i++)
		for (int j=0 ; j<m.mNumCols ; j++)
			ret.mM[i][j] = f * m.mM[i][j];
	return ret;
}

ostream& operator<<(ostream& os, const MatrixMxN& m)
{
	for (int i=0 ; i<m.mNumRows ; i++)
	{
		for (int j=0 ; j<m.mNumCols ; j++)
			os << m.mM[i][j] << ' ';
		os << '\n';
	}
	return os;
}

istream& operator>>(istream& is, MatrixMxN& m)
{
	for (int i=0 ; i<m.mNumRows ; i++)
	{
		for (int j=0 ; j<m.mNumCols ; j++)
			is >> m.mM[i][j];
	}
	return is;
}

void MatrixMxN::setCol(int j, const double rhs[])
{
	for(int i = 0; i < mNumRows; i++)
		mM[i][j] = rhs[i];
}

void MatrixMxN::setElementAt(int i, int j, double val)
{
	if ((i >= 0) && (i < mNumRows) && (j >= 0) && (j < mNumCols))
		mM[i][j] = val;
}

void MatrixMxN::setRow(int i, const double rhs[])
{
	for(int j = 0; j < mNumCols; j++)
		mM[i][j] = rhs[j];
}

MatrixMxN MatrixMxN::subMatrix(int skipRow, int skipCol) const
{
	MatrixMxN retval(mNumRows-1, mNumCols-1);
	if (skipRow < 0)
		skipRow = 0;
	else if (skipRow >= mNumRows)
		skipRow = mNumRows - 1;
	if (skipCol < 0)
		skipCol = 0;
	else if (skipCol >= mNumCols)
		skipCol = mNumCols - 1;

	int curRow = 0;
	for (int i=0 ; i<mNumRows ; i++)
	{
		if (i != skipRow)
		{
			int curCol = 0;
			for (int j=0 ; j<mNumCols ; j++)
				if (j != skipCol)
				{
					retval.mM[curRow][curCol] = mM[i][j];
					curCol++;
				}
			curRow++;
		}
	}
	return retval;
}

}
