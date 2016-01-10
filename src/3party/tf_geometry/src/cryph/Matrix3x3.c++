// Matrix3x3.c++ -- 3x3 transformation matrices
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <stdlib.h>
#include <math.h>
#include <iomanip>
using namespace std;

#include <cryph/Inline.h>
#include <cryph/Tolerances.h>

#include <cryph/Matrix3x3.h>
#include <cryph/Matrix4x4.h>
#include <cryph/Polynomial.h>

namespace cryph
{
// 1. Special Matrices
const Matrix3x3 Matrix3x3::IdentityMatrix =
	Matrix3x3(
		1.0 , 0.0 , 0.0 ,
		0.0 , 1.0 , 0.0 ,
		0.0 , 0.0 , 1.0
		);

const Matrix3x3 Matrix3x3::ZeroMatrix =
	Matrix3x3(
		0.0 , 0.0 , 0.0 ,
		0.0 , 0.0 , 0.0 ,
		0.0 , 0.0 , 0.0
		);

// 2. Return codes for extraction of unit axis vector & rotation angle
//    from 3x3 matrices that are 'supposed to be' orthogonal and
//    right-handed.

// 2.1: Abnormal internal errors that should never be returned:
const int	Matrix3x3::ImMDeterminantNotZero         = -20;
const int	Matrix3x3::CannotDetermineUnitAxisVector = -19;
const int	Matrix3x3::CosTermsNotEqual              = -18;
const int	Matrix3x3::SinTermsNotEqual              = -17;

// 2.2: "Normal" errors if user provides inappropriate matrix:
const int	Matrix3x3::NotOrthogonal                 =  -5;
const int	Matrix3x3::NotRightHanded                =  -4;

// 2.3: Successful extraction of unit axis vector and angle:
const int	Matrix3x3::Extracted_wTheta              =   0;

// END: public Global constants

// Other local private constants
static const int	DIM = 3;

Matrix3x3::Matrix3x3()
{
	this->copy(Matrix3x3::IdentityMatrix);
}

Matrix3x3::Matrix3x3(const Matrix3x3& M)
{
	this->copy(M);
}

Matrix3x3::Matrix3x3(
	double m11, double m12, double m13,
	double m21, double m22, double m23,
	double m31, double m32, double m33)
{
	mElem[0][0] = m11; mElem[0][1] = m12; mElem[0][2] = m13;
	mElem[1][0] = m21; mElem[1][1] = m22; mElem[1][2] = m23;
	mElem[2][0] = m31; mElem[2][1] = m32; mElem[2][2] = m33;
}

Matrix3x3::~Matrix3x3()
{
}

Matrix3x3 Matrix3x3::alignVectors(const AffVector& uFrom, const AffVector& uTo)
{
	AffVector	uHatF, uHatT;
	if (uFrom.normalizeToCopy(uHatF) < BasicDistanceTol)
		// zero-length vector -- cannot proceed
		return Matrix3x3::IdentityMatrix;

	if (uTo.normalizeToCopy(uHatT) < BasicDistanceTol)
		// zero-length vector -- cannot proceed
		return Matrix3x3::IdentityMatrix;

	AffVector rotAxis = uHatF.cross(uHatT);
	double cosine = uHatF.dot(uHatT);
	double sine   = rotAxis.normalize();
	double angle  = atan2(sine, cosine);
	if (sine < BasicDistanceTol)
	{
		if (cosine > 0.0)
			// the vectors are already coincident
			return Matrix3x3::IdentityMatrix;
		else
		{
			// must flip the vectors. We must generate a matrix that does a
			// rotation, however. That is, we cannot just return -I since that
			// is not a rigid transformation. This routine is called from
			// places where it is assumed that the matrix returned is a regular
			// rigid rotation. Hence we need to find some vector perpendicular
			// to both uHatF and uHatT. Given what we know about our clients,
			// we will try vectors in the order zu, yu, xu, and finally, an
			// arbitrary ector perpendicular to uHatF. Once we have the axis, we
			// will rotate by 180 degrees about it.
			if (fabsf(uHatF[DZ]) < BasicUnitTol)
				rotAxis = AffVector::zu;
			else if (fabsf(uHatF[DY]) < BasicUnitTol)
				rotAxis = AffVector::yu;
			else if (fabsf(uHatF[DX]) < BasicUnitTol)
				rotAxis = AffVector::xu;
			else
				uHatF.arbitraryNormal(rotAxis);
			angle = M_PI;
		}
	}
	return generalRotationRadians(rotAxis, angle);
}

Matrix3x3 Matrix3x3::alignVectors(const AffVector& uFrom, const AffVector& vFrom,
                          const AffVector& uTo,   const AffVector& vTo)
{
	AffVector	uHatT;
	if (uTo.normalizeToCopy(uHatT) < BasicDistanceTol)
		// zero-length vector -- cannot proceed
		return Matrix3x3::IdentityMatrix;

	Matrix3x3 part1 = alignVectors(uFrom, uTo);
	AffVector rotated_vFrom = part1 * vFrom;
	AffVector	parallel, vF, vT;
	uHatT.decompose(rotated_vFrom, parallel, vF);
	uHatT.decompose(vTo, parallel, vT);
	if (vF.parallelTo(vT))
		if (vF.dot(vT) > 0.0)
			// all done....
			return part1;
		else
			return generalRotationRadians(uHatT,M_PI) * part1;
	else
		return alignVectors(vF, vT) * part1;
}

int Matrix3x3::computeImMRows(Matrix3x3& ImM,
						AffVector rows[], int& r1, int& r2) const
{
	ImM = Matrix3x3::IdentityMatrix - *this;

	// Select the two rows of (I-M) that are most linearly independent.

	ImM.extractRows(rows[0],rows[1],rows[2]);

	// If any of the rows is zero, discard it and return the other two
	if (rows[0].lengthSquared() < BasicDistanceTol)
	{
		r1 = 1; r2 = 2;
	}
	else if (rows[1].lengthSquared() < BasicDistanceTol)
	{
		r1 = 0; r2 = 2;
	}
	else if (rows[2].lengthSquared() < BasicDistanceTol)
	{
		r1 = 0; r2 = 1;
	}
	else
	{
		double	testVal = fabs(rows[0].dot(rows[1]));
		int		ignoreRow = 2;
		for (int i=0 ; i<2 ; i++)
		{
			double tmp = fabs(rows[i].dot(rows[2]));
			if (tmp < testVal)
			{
				testVal = tmp;
				ignoreRow = 1 - i;
			}
		}
		switch (ignoreRow)
		{
			case 0:
				r1 = 1; r2 = 2;
				break;
			case 1:
				r1 = 0; r2 = 2;
				break;
			default: // must be "2"
				r1 = 0; r2 = 1;
		}
	}
	double	det = ImM.determinant();
	if (fabs(det) > BasicDistanceTol)
		return Matrix3x3::ImMDeterminantNotZero;
	return 0;
}

void Matrix3x3::copy(const Matrix3x3& rhs)
{
	for (int i=0 ; i<DIM ; i++)
		for (int j=0 ; j<DIM ; j++)
			this->mElem[i][j] = rhs.mElem[i][j];
}

Matrix3x3 Matrix3x3::crossProductMatrix(const AffVector& u)
{
	Matrix3x3	m = Matrix3x3::ZeroMatrix;

	/*********************/ m.mElem[0][1] = -u[DZ]; m.mElem[0][2] =  u[DY];
	m.mElem[1][0] =  u[DZ]; /*********************/ m.mElem[1][2] = -u[DX];
	m.mElem[2][0] = -u[DY]; m.mElem[2][1] =  u[DX]; /*********************/

	return m;
}

double Matrix3x3::determinant() const
{
	double	det = mElem[0][0] *
					(mElem[1][1]*mElem[2][2] - mElem[2][1]*mElem[1][2]);
	det -= mElem[0][1] * (mElem[1][0]*mElem[2][2] - mElem[2][0]*mElem[1][2]);
	det += mElem[0][2] * (mElem[1][0]*mElem[2][1] - mElem[2][0]*mElem[1][1]);

	return det;
}

int Matrix3x3::eigenValues(double lambda[], AffVector eigenV[], int* mult) const
{
	double	coeff[4] = {

	mElem[0][0]*mElem[1][1]*mElem[2][2] - mElem[0][0]*mElem[1][2]*mElem[2][1] -
	mElem[0][1]*mElem[1][0]*mElem[2][2] + mElem[0][1]*mElem[1][2]*mElem[2][0] +
	mElem[0][2]*mElem[1][0]*mElem[2][1] - mElem[0][2]*mElem[2][0]*mElem[1][1] ,

	mElem[1][2]*mElem[2][1] + mElem[0][1]*mElem[1][0] + mElem[0][2]*mElem[2][0]-
	mElem[0][0]*mElem[1][1] - mElem[0][0]*mElem[2][2] - mElem[1][1]*mElem[2][2],

	mElem[0][0] + mElem[1][1] + mElem[2][2] ,

	-1.0 };

	Polynomial p(coeff,3);
	Polynomial::Root ans[3];
	int nRoots = p.solve(ans);
	for (int i=0 ; i<nRoots ; i++)
	{
		if (mult != NULL)
			mult[i] = ans[i].multiplicity;
		lambda[i] = ans[i].value;
	}
	return nRoots;
}

double Matrix3x3::elementAt(int i, int j) const
{
	if ( (i < 3) && (j < 3) )
		return mElem[i][j];

	return -99999.0;
}

float* Matrix3x3::extractColMajor(float m[9]) const
{
	int		k = 0;
	for (int col=0 ; col<DIM ; col++)
		for (int row=0 ; row<DIM ; row++)
			m[k++] = static_cast<float>(this->mElem[row][col]);
	return &m[0];
}

double* Matrix3x3::extractColMajor(double m[9]) const
{
	int		k = 0;
	for (int col=0 ; col<DIM ; col++)
		for (int row=0 ; row<DIM ; row++)
			m[k++] = this->mElem[row][col];
	return &m[0];
}

float* Matrix3x3::extractRowMajor(float m[9]) const
{
	int		k = 0;
	for (int row=0 ; row<DIM ; row++)
		for (int col=0 ; col<DIM ; col++)
			m[k++] = static_cast<float>(this->mElem[row][col]);
	return &m[0];
}

double* Matrix3x3::extractRowMajor(double m[9]) const
{
	int		k = 0;
	for (int row=0 ; row<DIM ; row++)
		for (int col=0 ; col<DIM ; col++)
			m[k++] = this->mElem[row][col];
	return &m[0];
}

int Matrix3x3::extractAxisAngle(AffVector& w, double& theta) const
{
	if (!this->isOrthogonal())
		return Matrix3x3::NotOrthogonal;

	if (!this->isRightHanded())
		return Matrix3x3::NotRightHanded;

	int		pos;
	double	v = this->largestDiagonalElement(pos);
	if (v > (1.0-BasicDistanceTol))
		return this->extractPrimitiveAxisAngle(pos,w,theta);

	Matrix3x3	ImM;
	AffVector		rows[3];
	int			r1, r2;

	// compute the I-M matrix, extract its rows into "rows" and record in
	// "r1" and "r2" the indices of the two most linearly independent rows.
	// The following routine does this unconditionally, but returns a
	// negative code if the determinant of the I-M matrix is not zero.

	int code = this->computeImMRows(ImM, rows, r1, r2);
	if (code < 0)
		return code;

	// The desired axis vector is the cross product of the two most
	// linearly indepent rows as determined by "computeImMRows".
	// (Recall that the eigenvalue/eigenvector analysis says that the
	// dot product of these rows with the axis vector must be zero.)

	w = rows[r1].cross(rows[r2]);
	if (w.normalize() < BasicDistanceTol)
		return Matrix3x3::CannotDetermineUnitAxisVector;

	// Diagonal terms of the original matrix (in "*this") are of the form:
	//			"M[i][i] = cos(theta) + (1-cos(theta))wi^2)"
	// Hence, the trace(M) = M[0][0]+M[1][1]+M[2][2] = 2*cos(theta) + 1
	double cosTheta = 0.5 * (this->trace() - 1.0);

	// Off-diagonal terms of the original matrix are of the form:
	//			"M[i][j] = (1-cos(theta))*wi*wj + sin(theta)*Wij"
	// where W is the cross product matrix.
	// Here we choose (i,j) corresponding to the component of the axis
	// vector w with maximum absolute value.
	int maxWComponent;
	(void) w.maxAbsComponent(maxWComponent);

	// Solve the equation above for sin(theta):
	double sinTheta;
	switch (maxWComponent)
	{
		case DX:
			sinTheta = (mElem[2][1] - (1.0-cosTheta)*w[DY]*w[DZ]) / w[DX];
			break;
		case DY:
			sinTheta = (mElem[0][2] - (1.0-cosTheta)*w[DX]*w[DZ]) / w[DY];
			break;
		default: // must be DZ:
			sinTheta = (mElem[1][0] - (1.0-cosTheta)*w[DY]*w[DX]) / w[DZ];
	}

	theta = atan2(sinTheta,cosTheta);

	return Matrix3x3::Extracted_wTheta;
}

int Matrix3x3::extractPrimitiveAxisAngle(int pos, AffVector& w, double& theta) const
{
	// We will set "cosIndex1" and "cosIndex2" so that the cos terms will be at
	// M[cosIndex1][cosIndex1] and M[cosIndex2][cosIndex2]
	// int		cosIndex1, cosIndex2;
	// Since we only use the index as a sanity-check, however, we need only:
	int		other_cosIndex;

	// We will set "sinIndex1" and "sinIndex2" so that the sin terms are at
	// M[sinIndex1][sinIndex2] and M[sinIndex2][sinIndex1] AND SO THAT
	// M[sinIndex1][sinIndex2] is the location of +sin (as opposed to -sin).
	int		sinIndex1, sinIndex2;

	double		cosTheta, sinTheta;
	switch (pos)
	{
		case X:
			cosTheta = mElem[1][1];
			// cosIndex1 = 1; cosIndex2 = 2;
			other_cosIndex = 2;
			sinTheta = mElem[2][1];
			sinIndex1 = 2; sinIndex2 = 1;
			w = AffVector::xu;
			break;
		case Y:
			cosTheta = mElem[0][0];
			// cosIndex1 = 0; cosIndex2 = 2;
			other_cosIndex = 2;
			sinTheta = mElem[0][2];
			sinIndex1 = 0; sinIndex2 = 2;
			w = AffVector::yu;
			break;
		default: // must be Z:
			cosTheta = mElem[0][0];
			// cosIndex1 = 0; cosIndex2 = 1;
			other_cosIndex = 1;
			sinTheta = mElem[1][0];
			sinIndex1 = 1; sinIndex2 = 0;
			w = AffVector::zu;
	}

	// Just to make sure....
	if (!equalScalars(cosTheta,
	                  mElem[other_cosIndex][other_cosIndex],BasicUnitTol))
		return Matrix3x3::CosTermsNotEqual;
	if (!equalScalars(sinTheta,
	                 -mElem[sinIndex2][sinIndex1],BasicUnitTol))
		return Matrix3x3::SinTermsNotEqual;

	// Compute the angle
	theta = atan2(sinTheta,cosTheta);

	return Matrix3x3::Extracted_wTheta;
}

void Matrix3x3::extractRows(AffVector& row1, AffVector& row2, AffVector& row3) const
{
	row1 = AffVector(mElem[0][0], mElem[0][1], mElem[0][2]);
	row2 = AffVector(mElem[1][0], mElem[1][1], mElem[1][2]);
	row3 = AffVector(mElem[2][0], mElem[2][1], mElem[2][2]);
}

Matrix3x3 Matrix3x3::generalRotationRadians(const AffVector& rotationAxis, double angle)
{
	AffVector	w;
	if (rotationAxis.normalizeToCopy(w) < BasicDistanceTol)
		// zero-length vector -- cannot proceed
		return Matrix3x3::IdentityMatrix;

	Matrix3x3 T = tensorProductMatrix(w,w);
	Matrix3x3	Xprod = crossProductMatrix(w);

	double c = cos(angle); double s = sin(angle);

	return c*Matrix3x3::IdentityMatrix + (1.0-c)*T + s*Xprod;
}
	
bool Matrix3x3::inverse(Matrix3x3& mInv) const
{
	Matrix4x4 temp(*this, AffVector::zeroVector);
	Matrix4x4 mInv4x4;
	if (temp.inverse(mInv4x4))
	{
		mInv = mInv4x4.subMatrix(3,3);
		return true;
	}
	return false;
}

bool Matrix3x3::isOrthogonal() const
{
	AffVector	u, v, w;
	extractRows(u,v,w);
	if (fabs(1.0-u.length()) < BasicDistanceTol)
		if (fabs(1.0-v.length()) < BasicDistanceTol)
			if (fabs(1.0-w.length()) < BasicDistanceTol)
			{
				if (u.dot(v) < BasicDistanceTol)
					if (u.dot(w) < BasicDistanceTol)
						if (v.dot(w) < BasicDistanceTol)
							return true;
			}
	return false;
}

bool Matrix3x3::isRightHanded() const
{
	AffVector	u, v, w;
	extractRows(u,v,w);
	if (w.dot(u.cross(v)) > 0.0)
		return true;
	return false;
}

double Matrix3x3::largestDiagonalElement(int& pos) const
{
	double largestVal = mElem[0][0];
	pos = 0;
	for (int i=0 ; i<DIM ; i++)
		if (mElem[i][i] > largestVal)
		{
			largestVal = mElem[i][i];
			pos = i;
		}
	return largestVal;
}

Matrix3x3 Matrix3x3::mirrorMatrix(const AffVector& mirrorPlaneNormal)
{
	AffVector	n;
	if (mirrorPlaneNormal.normalizeToCopy(n) < BasicDistanceTol)
		// zero-length vector -- cannot proceed
		return Matrix3x3::IdentityMatrix;

	Matrix3x3 T = tensorProductMatrix(n,n);

	return Matrix3x3::IdentityMatrix - 2.0*T;
}

void Matrix3x3::multiply(const double a[], double b[]) const
{
	for (int i=0 ; i<3 ; i++)
	{
		double sum = 0.0;
		for (int j=0 ; j<3 ; j++)
			sum += mElem[i][j]*a[j];
		b[i] = sum;
	}
}

Matrix3x3 Matrix3x3::operator=(const Matrix3x3& rhs)
{
	this->copy(rhs);
	return *this;
}

Matrix3x3 Matrix3x3::operator*=(const Matrix3x3& rhs)
{
	*this = (*this) * rhs;
	return *this;
}

Matrix3x3 Matrix3x3::operator*=(double f)
{
	for (int i=0 ; i<DIM ; i++)
		for (int j=0 ; j<DIM ; j++)
			this->mElem[i][j] *= f;
	return *this;
}

Matrix3x3 Matrix3x3::operator+=(const Matrix3x3& rhs)
{
	for (int i=0 ; i<DIM ; i++)
		for (int j=0 ; j<DIM ; j++)
			this->mElem[i][j] += rhs.mElem[i][j];
	return *this;
}

AffPoint Matrix3x3::operator*(const AffPoint& p) const
{
	double	c[DIM] = { p[X] , p[Y] , p[Z] };
	double	d[DIM];

	for (int i=0 ; i<DIM ; i++)
	{
		double sum = 0.0;
		for (int j=0 ; j<DIM ; j++)
			sum += (this->mElem[i][j] * c[j]);
		d[i] = sum;
	}
	return AffPoint(d[0],d[1],d[2]);
}

AffVector Matrix3x3::operator*(const AffVector& v) const
{
	double	c[DIM] = { v[DX] , v[DY] , v[DZ] };
	double	d[DIM];

	for (int i=0 ; i<DIM ; i++)
	{
		double sum = 0.0;
		for (int j=0 ; j<DIM ; j++)
			sum += (this->mElem[i][j] * c[j]);
		d[i] = sum;
	}
	return AffVector(d[0],d[1],d[2]);
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3& m2) const
{
	Matrix3x3 mOut;

	for (int i=0 ; i<DIM ; i++)
		for (int j=0 ; j<DIM ; j++)
		{
			double sum = 0.0;
			for (int k=0 ; k<DIM ; k++)
				sum += mElem[i][k]*m2.mElem[k][j];
			mOut.mElem[i][j] = sum;
		}

	return mOut;
}

Matrix3x3 Matrix3x3::operator+(const Matrix3x3& m2) const
{
	Matrix3x3	result(*this);

	for (int i=0 ; i<DIM ; i++)
		for (int j=0 ; j<DIM ; j++)
			result.mElem[i][j] += m2.mElem[i][j];
	return result;
}

Matrix3x3 Matrix3x3::operator-(const Matrix3x3& m2) const
{
	Matrix3x3	result(*this);

	for (int i=0 ; i<DIM ; i++)
		for (int j=0 ; j<DIM ; j++)
			result.mElem[i][j] -= m2.mElem[i][j];
	return result;
}

Matrix3x3 operator*(double f, const Matrix3x3& m) // friend function; not a method
{
	Matrix3x3	result(m);

	for (int i=0 ; i<DIM ; i++)
		for (int j=0 ; j<DIM ; j++)
			result.mElem[i][j] *= f;
	return result;
}

ostream& operator<<(ostream& os, const Matrix3x3& m)
{
	for (int i=0 ; i<DIM ; i++)
	{
		for (int j=0 ; j<DIM ; j++)
		{
			os << m.mElem[i][j] << ' ';
		}
	}
	return os;
}

istream& operator>>(istream& is, Matrix3x3& m)
{
	for (int i=0 ; i<DIM ; i++)
	{
		for (int j=0 ; j<DIM ; j++)
		{
			is >> m.mElem[i][j];
		}
	}

	return is;
}

Matrix3x3 Matrix3x3::scale(double sx, double sy, double sz)
{
	Matrix3x3	m = Matrix3x3::IdentityMatrix;

	m.mElem[0][0] = sx;
	m.mElem[1][1] = sy;
	m.mElem[2][2] = sz;

	return m;
}

void Matrix3x3::setElementAt(int i, int j, double newValue)
{
	if ( (i < 3) && (j < 3) )
		mElem[i][j] = newValue;
}

Matrix3x3 Matrix3x3::shear(const AffVector& n, const AffVector& u, double f)
{
	Matrix3x3   m = Matrix3x3::IdentityMatrix + f*tensorProductMatrix(u,n);
	return m;
}

Matrix3x3 Matrix3x3::tensorProductMatrix(const AffVector& u, const AffVector& v)
{
	double U[DIM] = { u[DX] , u[DY] , u[DZ] };
	double V[DIM] = { v[DX] , v[DY] , v[DZ] };

	Matrix3x3	m(Matrix3x3::ZeroMatrix);

	for (int i=0 ; i<DIM ; i++)
		for (int j=0 ; j<DIM ; j++)
			m.mElem[i][j] = U[i]*V[j];

	return m;
}

double Matrix3x3::trace() const
{
	return mElem[0][0] + mElem[1][1] + mElem[2][2];
}

void Matrix3x3::transpose()
{
	swap2(mElem[0][1],mElem[1][0]);
	swap2(mElem[0][2],mElem[2][0]);
	swap2(mElem[1][2],mElem[2][1]);
}

Matrix3x3 Matrix3x3::xRotationDegrees(double angle)
{
	double radians = degreesToRadians(angle);
	double c = cos(radians);
	double s = sin(radians);

	Matrix3x3	m(Matrix3x3::IdentityMatrix);
	m.mElem[1][1] =  c; m.mElem[1][2] = -s;
	m.mElem[2][1] =  s; m.mElem[2][2] =  c;

	return m;
}

Matrix3x3 Matrix3x3::xRotationRadians(double angle)
{
	double c = cos(angle);
	double s = sin(angle);

	Matrix3x3	m(Matrix3x3::IdentityMatrix);
	m.mElem[1][1] =  c; m.mElem[1][2] = -s;
	m.mElem[2][1] =  s; m.mElem[2][2] =  c;

	return m;
}

Matrix3x3 Matrix3x3::yRotationDegrees(double angle)
{
	double radians = degreesToRadians(angle);
	double c = cos(radians);
	double s = sin(radians);

	Matrix3x3	m(Matrix3x3::IdentityMatrix);
	m.mElem[0][0] =  c; m.mElem[0][2] =  s;
	m.mElem[2][0] = -s; m.mElem[2][2] =  c;

	return m;
}

Matrix3x3 Matrix3x3::yRotationRadians(double angle)
{
	double c = cos(angle);
	double s = sin(angle);

	Matrix3x3	m(Matrix3x3::IdentityMatrix);
	m.mElem[0][0] =  c; m.mElem[0][2] =  s;
	m.mElem[2][0] = -s; m.mElem[2][2] =  c;

	return m;
}

Matrix3x3 Matrix3x3::zRotationDegrees(double angle)
{
	double radians = degreesToRadians(angle);
	double c = cos(radians);
	double s = sin(radians);

	Matrix3x3	m(Matrix3x3::IdentityMatrix);
	m.mElem[0][0] =  c; m.mElem[0][1] = -s;
	m.mElem[1][0] =  s; m.mElem[1][1] =  c;

	return m;
}

Matrix3x3 Matrix3x3::zRotationRadians(double angle)
{
	double c = cos(angle);
	double s = sin(angle);

	Matrix3x3	m(Matrix3x3::IdentityMatrix);
	m.mElem[0][0] =  c; m.mElem[0][1] = -s;
	m.mElem[1][0] =  s; m.mElem[1][1] =  c;

	return m;
}

}
