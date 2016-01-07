// Matrix4x4.c++ -- 4x4 transformation matrices
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <stdlib.h>
#include <math.h>

#include <iomanip>
using namespace std;

#include <cryph/Inline.h>
#include <cryph/Tolerances.h>

#include <cryph/Matrix4x4.h>
#include <cryph/ProjPoint.h>
#include <cryph/ViewVolume.h>

namespace cryph
{

// ---------- public Global constants

// 1. Special Matrices
const Matrix4x4 Matrix4x4::IdentityMatrix =
	Matrix4x4(
		1.0 , 0.0 , 0.0 , 0.0 ,
		0.0 , 1.0 , 0.0 , 0.0 ,
		0.0 , 0.0 , 1.0 , 0.0 ,
		0.0 , 0.0 , 0.0 , 1.0
		);

const Matrix4x4 Matrix4x4::ZeroMatrix =
	Matrix4x4(
		0.0 , 0.0 , 0.0 , 0.0 ,
		0.0 , 0.0 , 0.0 , 0.0 ,
		0.0 , 0.0 , 0.0 , 0.0 ,
		0.0 , 0.0 , 0.0 , 0.0
		);

// 2. Return codes for extraction of base point, unit axis vector,
//    and rotation angle from 4x4 matrices that are 'supposed to
//    describe' affine transformations whose upper 3x3 matrix is
//    'supposed to be' orthogonal and right-handed. (See the Matrix3x3
//    method "extractAxisAngle".)

// 2.1: Abnormal internal errors that should never be returned:
const int	Matrix4x4::InternalBasePointComputationError = -30;

// 2.2: "Normal" errors if user provides inappropriate matrix:
const int	Matrix4x4::NotAffine                         = -10;
// (Note that the Matrix3x3 error codes may be returned as well.)

// 2.3: Successful extraction of unit axis vector and angle:
//      If the 3x3 extraction of (unit axis vector, angle) is successful,
//      then the 4x4 utility tries to determine a base point. If
//      successful, then "Matrix4x4::Extracted_BwTheta" is returned,
//      otherwise the 3x3 code ("Matrix3x3::Extracted_wTheta") is returned,
//      signifying that the (unit axis vector, angle) information was
//      successfully computed and returned, but only an approximation to
//      a rotation axis base point was able to be determined. In this case,
//      'postTranslation' will contain a non-zero vector which, if added to
//      a point after rotation, will yield the correct point.
const int	Matrix4x4::Extracted_BwTheta                 = 1;

// END: public Global constants

// Following used simply to make it obvious where we are assuming one
// dimension or another:
const int DIM3 = 3;
const int DIM4 = 4;

Matrix4x4::Matrix4x4()
{
	this->copy(Matrix4x4::IdentityMatrix);
}

Matrix4x4::Matrix4x4(const Matrix4x4& m)
{
	this->copy(m);
}

Matrix4x4::Matrix4x4(
	double m11, double m12, double m13, double m14,
	double m21, double m22, double m23, double m24,
	double m31, double m32, double m33, double m34,
	double m41, double m42, double m43, double m44)
{
	mElem[0][0] = m11; mElem[0][1] = m12; mElem[0][2] = m13; mElem[0][3] = m14;
	mElem[1][0] = m21; mElem[1][1] = m22; mElem[1][2] = m23; mElem[1][3] = m24;
	mElem[2][0] = m31; mElem[2][1] = m32; mElem[2][2] = m33; mElem[2][3] = m34;
	mElem[3][0] = m41; mElem[3][1] = m42; mElem[3][2] = m43; mElem[3][3] = m44;
}

Matrix4x4::Matrix4x4(const Matrix3x3& M)
{
	AffVector	r1, r2, r3;
	M.extractRows(r1,r2,r3);

	mElem[0][0] = r1[0]; mElem[0][1] = r1[1];
			mElem[0][2] = 0.0; mElem[0][3] = r1[2];
	mElem[1][0] = r2[0]; mElem[1][1] = r2[1];
			mElem[1][2] = 0.0; mElem[1][3] = r2[2];
	mElem[2][0] = 0.0;   mElem[2][1] = 0.0;
			mElem[2][2] = 1.0; mElem[2][3] = 0.0;
	mElem[3][0] = r3[0]; mElem[3][1] = r3[1];
			mElem[3][2] = 0.0; mElem[3][3] = r3[2];
}

// From an Affine transformation spec: (M, t). 4th row gets (0,0,0,1)
Matrix4x4::Matrix4x4(const Matrix3x3& M, const AffVector& t)
{
	installMt(M,t);
}

// From an affine matrix M and a fixed point:
Matrix4x4::Matrix4x4(const Matrix3x3& M, const AffPoint& FixedPoint)
{
	AffVector	t = FixedPoint - M*FixedPoint;
	installMt(M,t);
}

// From an affine matrix M and a point whose pre- and post-image are known
Matrix4x4::Matrix4x4(const Matrix3x3& M,
				const AffPoint& PreImage, const AffPoint& PostImage)
{
	AffVector t = PostImage - M*PreImage;
	installMt(M,t);
}

Matrix4x4::~Matrix4x4()
{
}

void Matrix4x4::copy(const Matrix4x4& rhs)
{
	for (int i=0 ; i<DIM4 ; i++)
		for (int j=0 ; j<DIM4 ; j++)
			this->mElem[i][j] = rhs.mElem[i][j];
}

double Matrix4x4::determinant() const
{
	double det = 0.0;
	double sign = 1.0;
	for (int col=0 ; col<4 ; col++)
	{
		Matrix3x3 m3x3 = subMatrix(0,col);
		det += sign*elementAt(0,col)*m3x3.determinant();
		sign = -sign;
	}
	return det;
}

int Matrix4x4::determineBasePoint(AffVector ImMRows[], int maxWCompLoc,
				int r1, int r2,
				const AffVector& trans, AffPoint& B)
{
	// Last stage of extracting a rotation axis & angle from a 4x4 matrix.
	// The two rows (I - M) indicated by "r1" and "r2" are linearly
	// independent. We know that (I - M)B = trans. The input "maxWCompLoc" is
	// the index of the component of the rotation axis with largest absolute
	// value. Hence we can set B[maxWCompLoc] to anything and solve for the
	// other two coordinates.
	// Here we use the constant "Bfixed" to allow easy experimentation.

	const double Bfixed = 0.0;

	// We therefore have the following two equations:
//   ImMRows[r1][0]*Bx + ImMRows[r1][1]*By + ImMRows[r1][2]*Bz - trans[r1] = 0
//   ImMRows[r2][0]*Bx + ImMRows[r2][1]*By + ImMRows[r2][2]*Bz - trans[r2] = 0

	// Assuming that B[maxWCompLoc] is "Bfixed", we get two equations in two
	// unknowns that can be interpreted as:
	//   S.Bprime = 0
	//   T.Bprime = 0
	// where:
	// 		S = (ImMRows[r1][j], ImMRows[r1][k],
	//					Bfixed*ImMRows[r1][maxWCompLoc]-trans[r1])
	//		T = (ImMRows[r2][j], ImMRows[r2][k],
	//					Bfixed*ImMRows[r2][maxWCompLoc]-trans[r2])
	//		Bprime = (Bj, Bk, 1).
	//		"j" and "k" denote the two components other than "maxWCompLoc"

	// Bprime can be computed as the cross product: S x T. This will, in
	// general, yield a "Bprime" with some value other than "1" in the third
	// spot, hence we scale the resulting vector to get a "1" in that spot.

	AffVector	S, T;
	switch (maxWCompLoc)
	{
		case DX:
			S = AffVector(ImMRows[r1][DY],ImMRows[r1][DZ],
						Bfixed*ImMRows[r1][DX]-trans[r1]);
			T = AffVector(ImMRows[r2][DY],ImMRows[r2][DZ],
						Bfixed*ImMRows[r2][DX]-trans[r2]);
			break;
		case DY:
			S = AffVector(ImMRows[r1][DX],ImMRows[r1][DZ],
						Bfixed*ImMRows[r1][DY]-trans[r1]);
			T = AffVector(ImMRows[r2][DX],ImMRows[r2][DZ],
						Bfixed*ImMRows[r2][DY]-trans[r2]);
			break;
		default: // must be DZ:
			S = AffVector(ImMRows[r1][DX],ImMRows[r1][DY],
						Bfixed*ImMRows[r1][DZ]-trans[r1]);
			T = AffVector(ImMRows[r2][DX],ImMRows[r2][DY],
						Bfixed*ImMRows[r2][DZ]-trans[r2]);
	}
	AffVector Bprime = AffVector::cross(S,T);
	if (fabs(Bprime[DZ]) < BasicDistanceTol)
		return Matrix4x4::InternalBasePointComputationError;

	switch (maxWCompLoc)
	{
		case DX:
			B = AffPoint(Bfixed, Bprime[DX]/Bprime[DZ], Bprime[DY]/Bprime[DZ]);
			break;
		case DY:
			B = AffPoint(Bprime[DX]/Bprime[DZ], Bfixed, Bprime[DY]/Bprime[DZ]);
			break;
		default: // must be DZ:
			B = AffPoint(Bprime[DX]/Bprime[DZ], Bprime[DY]/Bprime[DZ], Bfixed);
	}
	return Matrix4x4::Extracted_BwTheta;
}

double Matrix4x4::elementAt(int i, int j) const
{
	if ( (i < 4) && (j < 4) )
		return mElem[i][j];

	return -99999.0;
}

void Matrix4x4::extractAffineMt(Matrix3x3& M, AffVector& t) const
{
	double	col4[3];

	for (int i=0 ; i<DIM3 ; i++)
	{
		for (int j=0 ; j<DIM3 ; j++)
			M.mElem[i][j] = this->mElem[i][j];

		col4[i] = this->mElem[i][DIM4-1];
	}
	t = AffVector(col4[0],col4[1],col4[2]);
}

int Matrix4x4::extractAxisAngle(AffPoint& B, AffVector& w, double& theta,
				AffVector& postTranslation) const
{
	if (this->isAffineTransformation() != 1)
		return Matrix4x4::NotAffine;

	Matrix3x3	M3x3;
	AffVector		trans;
	this->extractAffineMt(M3x3,trans);

	int outcome = M3x3.extractAxisAngle(w,theta);
	if (outcome == Matrix3x3::Extracted_wTheta)
	{
		// if the extracted angle is "0", then the base point must be (0,0,0)
		if (fabs(theta) < BasicAngleTol)
		{
			B = AffPoint::origin;
			return Matrix4x4::Extracted_BwTheta;
		}

		// Otherwise we need to do some work!!
		int		maxWComponent;
		(void) w.maxAbsComponent(maxWComponent);

		// Attempt to determine a base point for the extracted rotation axis
		// that would give rise to the extracted affine translation vector.

		Matrix3x3	ImM;
		AffVector		rows[3];
		int			r1, r2;

		M3x3.computeImMRows(ImM,rows,r1,r2);
		outcome = Matrix4x4::determineBasePoint(
							rows,maxWComponent,r1,r2,trans,B);

		if (outcome == Matrix4x4::Extracted_BwTheta)
		{
			// We have computed a possible base point based on the two
			// linearly independent rows. Unfortunately, it is not always
			// possible to find a base point to match an arbitrary translation
			// term. We compute and return in "postTranslation" the residula
			// translation vector. The interpretation is then: "(B,w,theta)
			// FOLLOWED BY translation by "postTranslation" is equivalent
			// to the original matrix.

			AffVector testTrans = ImM * B;
			postTranslation = trans - testTrans;
		}
	}
	return outcome;
}

float* Matrix4x4::extractColMajor(float m[16]) const
{
	int		k = 0;
	for (int col=0 ; col<DIM4 ; col++)
		for (int row=0 ; row<DIM4 ; row++)
			m[k++] = static_cast<float>(this->mElem[row][col]);
	return &m[0];
}

double* Matrix4x4::extractColMajor(double m[16]) const
{
	int		k = 0;
	for (int col=0 ; col<DIM4 ; col++)
		for (int row=0 ; row<DIM4 ; row++)
			m[k++] = this->mElem[row][col];
	return &m[0];
}

float* Matrix4x4::extractRowMajor(float m[16]) const
{
	int		k = 0;
	for (int row=0 ; row<DIM4 ; row++)
		for (int col=0 ; col<DIM4 ; col++)
			m[k++] = static_cast<float>(this->mElem[row][col]);
	return &m[0];
}

double* Matrix4x4::extractRowMajor(double m[16]) const
{
	int		k = 0;
	for (int row=0 ; row<DIM4 ; row++)
		for (int col=0 ; col<DIM4 ; col++)
			m[k++] = this->mElem[row][col];
	return &m[0];
}

Matrix4x4 Matrix4x4::fromColMajor(const float* m)
{
	Matrix4x4 M;
	int k = 0;
	for (int col=0 ; col<DIM4 ; col++)
		for (int row=0 ; row<DIM4 ; row++)
			M.mElem[row][col] = m[k++];
	return M;
}

Matrix4x4 Matrix4x4::fromColMajor(const double* m)
{
	Matrix4x4 M;
	int k = 0;
	for (int col=0 ; col<DIM4 ; col++)
		for (int row=0 ; row<DIM4 ; row++)
			M.mElem[row][col] = m[k++];
	return M;
}

Matrix4x4 Matrix4x4::fromRowMajor(const float* m)
{
	Matrix4x4 M;
	int k = 0;
	for (int row=0 ; row<DIM4 ; row++)
		for (int col=0 ; col<DIM4 ; col++)
			M.mElem[row][col] = m[k++];
	return M;
}

Matrix4x4 Matrix4x4::fromRowMajor(const double* m)
{
	Matrix4x4 M;
	int k = 0;
	for (int row=0 ; row<DIM4 ; row++)
		for (int col=0 ; col<DIM4 ; col++)
			M.mElem[row][col] = m[k++];
	return M;
}

bool Matrix4x4::getECvw(const cryph::AffPoint& eye,
        const cryph::AffPoint& center, const cryph::AffVector& up,
		cryph::AffVector& v, cryph::AffVector& w)
{
	w = eye - center;
	if (w.normalize() < BasicDistanceTol)
	{
		std::cerr << "Matrix4x4::getECvw: eye and center are coincident.\n";
		return false;
	}
	AffVector par;
	w.decompose(up,par, v);
	if (v.normalize() < BasicUnitTol)
	{
		std::cerr << "Matrix4x4::getECvw: 'up' is zero or parallel to line of sight.\n";
		return false;
	}
	return true;

}

void Matrix4x4::installMt(const Matrix3x3& M, const AffVector& t)
{
	for (int i=0 ; i<DIM3 ; i++)
		for (int j=0 ; j<DIM3 ; j++)
			this->mElem[i][j] = M.mElem[i][j];

	double	T[3] = { t[DX], t[DY], t[DZ] };

	for (int k=0 ; k<DIM3 ; k++)
	{
		this->mElem[k][DIM4-1] = T[k];
		this->mElem[DIM4-1][k] = 0.0;
	}
	this->mElem[DIM4-1][DIM4-1] = 1.0;
}

// Not very numerically reliable. Works OK with friendly matrices.
bool Matrix4x4::inverse(Matrix4x4& mInv) const
{
	double detA = determinant();
	if (fabs(detA) < 1.0e-9)
		return false;

	Matrix3x3 subM;
	double startRowSign = 1.0;
	for (int i=0 ; i<4 ; i++)
	{
		double sign = startRowSign;
		startRowSign = -startRowSign;
		for (int j=0 ; j<4 ; j++)
		{
			subM = subMatrix(j,i); // note index order here and in next line
			mInv.mElem[i][j] = sign * subM.determinant() / detA;
			sign = -sign;
		}
	}
	return true;
}

int Matrix4x4::isAffineTransformation() const
{
	int flag = 0;
	double sum = square(this->mElem[3][0]);
	sum += square(this->mElem[3][1]);
	sum += square(this->mElem[3][2]);
	if (sum > BasicDistanceTol)
		flag = 2;
	if (fabs(1.0-this->mElem[3][3]) < BasicDistanceTol)
		return flag |= 1;
	return flag;
}

Matrix4x4 Matrix4x4::lookAt(const cryph::AffPoint& eye,
		const cryph::AffPoint& center, const cryph::AffVector& up)
{
	AffVector v, w;
	if (!getECvw(eye, center, up, v, w))
		return IdentityMatrix;
	AffVector u = v.cross(w);
	u.normalize();
	double tx = -u.dot(eye);
	double ty = -v.dot(eye);
	double tz = -w.dot(eye);
	return Matrix4x4(
	               u[DX],u[DY],u[DZ], tx,
				   v[DX],v[DY],v[DZ], ty,
				   w[DX],w[DY],w[DZ], tz,
	                 0.0,  0.0,  0.0, 1.0);
}

void Matrix4x4::multiply(const double a[], double b[], int nElements) const
{
	if (nElements < 1)
		return;
	if (nElements > 4)
		nElements = 4;
	for (int i=0 ; i<nElements ; i++)
	{
		double sum = 0.0;
		for (int j=0 ; j<4 ; j++)
		{
			double aj = 0.0;
			if (j < nElements)
				aj = a[j];
			else if (j == 3)
				aj = 1.0; // w
			sum += mElem[i][j]*aj;
		}
		b[i] = sum;
	}
}

void Matrix4x4::multiply(const float a[], float b[], int nElements) const
{
	if (nElements < 1)
		return;
	if (nElements > 4)
		nElements = 4;
	for (int i=0 ; i<nElements ; i++)
	{
		float sum = 0.0;
		for (int j=0 ; j<4 ; j++)
		{
			double aj = 0.0;
			if (j < nElements)
				aj = a[j];
			else if (j == 3)
				aj = 1.0; // w
			sum += static_cast<float>(mElem[i][j]*aj);
		}
		b[i] = sum;
	}
}

Matrix4x4 Matrix4x4::oblique(double zpp, double xmin, double xmax,
	                         double ymin, double ymax, double zmin, double zmax,
                             const cryph::AffVector& projDir)
{
	if ((xmin >= xmax) || (ymin >= ymax) || (zmin >= zmax))
	{
		std::cerr << "Matrix4x4::oblique: one or more min values >= max.\n";
		return IdentityMatrix;
	}
	double pDZ = projDir[DZ];
	if (fabs(pDZ) < BasicUnitTol)
	{
		std::cerr << "Matrix4x4::oblique: invalid projection direction: " << projDir << '\n';
		return IdentityMatrix;
	}
	double dx = xmax - xmin;
	double dy = ymax - ymin;
	double dz = zmin - zmax; // note swapped order
	double pDX = projDir[DX];
	double pDY = projDir[DY];
	return cryph::Matrix4x4(
		2.0/dx, 0.0, -2.0*pDX/(pDZ*dx), 2.0*zpp*pDX/(pDZ*dx) - (xmin+xmax)/dx,
		0.0, 2.0/dy, -2.0*pDY/(pDZ*dy), 2.0*zpp*pDY/(pDZ*dy) - (ymin+ymax)/dy,
		0.0, 0.0, 2.0/dz, -(zmin + zmax)/dz,
		0.0, 0.0, 0.0, 1.0
	);
}

Matrix4x4 Matrix4x4::operator=(const Matrix4x4& rhs)
{
	this->copy(rhs);
	return *this;
}

Matrix4x4 Matrix4x4::operator*=(const Matrix4x4& rhs)
{
	*this = (*this) * rhs;
	return *this;
}

Matrix4x4 Matrix4x4::operator*=(double f)
{
	for (int i=0 ; i<DIM4 ; i++)
		for (int j=0 ; j<DIM4 ; j++)
			this->mElem[i][j] *= f;
	return *this;
}

Matrix4x4 Matrix4x4::operator+=(const Matrix4x4& rhs)
{
	for (int i=0 ; i<DIM4 ; i++)
		for (int j=0 ; j<DIM4 ; j++)
			this->mElem[i][j] += rhs.mElem[i][j];
	return *this;
}

AffPoint Matrix4x4::operator*(const AffPoint& p) const
{
	double	c[4] = { p[X] , p[Y] , p[Z], 1.0 };
	double	d[4];

	for (int i=0 ; i<4 ; i++)
	{
		double sum = 0.0;
		for (int j=0 ; j<4 ; j++)
			sum += (this->mElem[i][j] * c[j]);
		d[i] = sum;
	}

	if (fabs(d[3]) > BasicDistanceTol)
		return AffPoint(d[0]/d[3],d[1]/d[3],d[2]/d[3]);

	return AffPoint(d[0],d[1],d[2]);
}

ProjPoint Matrix4x4::operator*(const ProjPoint& p) const
{
	double	c[4] = { p[X] , p[Y] , p[Z], p[W] };
	double	d[4];

	for (int i=0 ; i<4 ; i++)
	{
		double sum = 0.0;
		for (int j=0 ; j<4 ; j++)
			sum += (this->mElem[i][j] * c[j]);
		d[i] = sum;
	}

	return ProjPoint(d);
}

AffVector Matrix4x4::operator*(const AffVector& v) const
{
	double	c[3] = { v[DX] , v[DY] , v[DZ] };
	double	d[3];

	for (int i=0 ; i<3 ; i++)
	{
		double sum = 0.0;
		for (int j=0 ; j<3 ; j++)
			sum += (this->mElem[i][j] * c[j]);
		d[i] = sum;
	}
	return AffVector(d[0],d[1],d[2]);
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4& m2) const
{
	Matrix4x4 mOut;

	for (int i=0 ; i<DIM4 ; i++)
		for (int j=0 ; j<DIM4 ; j++)
		{
			double sum = 0.0;
			for (int k=0 ; k<DIM4 ; k++)
				sum += mElem[i][k]*m2.mElem[k][j];
			mOut.mElem[i][j] = sum;
		}

	return mOut;
}

Matrix4x4 Matrix4x4::operator+(const Matrix4x4& m2) const
{
	Matrix4x4	result(*this);

	for (int i=0 ; i<DIM4 ; i++)
		for (int j=0 ; j<DIM4 ; j++)
			result.mElem[i][j] += m2.mElem[i][j];
	return result;
}

Matrix4x4 Matrix4x4::operator-(const Matrix4x4& m2) const
{
	Matrix4x4	result(*this);

	for (int i=0 ; i<DIM4 ; i++)
		for (int j=0 ; j<DIM4 ; j++)
			result.mElem[i][j] -= m2.mElem[i][j];
	return result;
}

Matrix4x4 operator*(double f, const Matrix4x4& m) // external friend function
{
	Matrix4x4	result(m);

	for (int i=0 ; i<DIM4 ; i++)
		for (int j=0 ; j<DIM4 ; j++)
			result.mElem[i][j] *= f;
	return result;
}

ostream& operator<<(ostream& os, const Matrix4x4& m)
{
	for (int i=0 ; i<DIM4 ; i++)
	{
		for (int j=0 ; j<DIM4 ; j++)
		{
			os << m.mElem[i][j] << ' ';
		}
	}
	return os;
}

istream& operator>>(istream& is, Matrix4x4& m)
{
	for (int i=0 ; i<DIM4 ; i++)
	{
		for (int j=0 ; j<DIM4 ; j++)
		{
			is >> m.mElem[i][j];
		}
	}
	return is;
}

Matrix4x4 Matrix4x4::orthogonal(double xmin, double xmax, double ymin,
	                            double ymax, double zmin, double zmax)
{
	if ((xmin >= xmax) || (ymin >= ymax) || (zmin >= zmax))
	{
		std::cerr << "Matrix4x4::orthogonal: one or more min values >= max.\n";
		return IdentityMatrix;
	}
	return Matrix4x4(
		2.0/(xmax-xmin), 0.0, 0.0, -(xmax+xmin)/(xmax-xmin),
		0.0, 2.0/(ymax-ymin), 0.0, -(ymax+ymin)/(ymax-ymin),
		0.0, 0.0, -2.0/(zmax-zmin), (zmax+zmin)/(zmax-zmin),
		0.0, 0.0, 0.0, 1.0
	);
}

Matrix4x4 Matrix4x4::perspective(double zpp, double xmin, double xmax,
	                         double ymin, double ymax, double zmin, double zmax)
{
	if ((xmin >= xmax) || (ymin >= ymax) || (zmin >= zmax))
	{
		std::cerr << "Matrix4x4::perspective: one or more min values >= max.\n";
		return IdentityMatrix;
	}
	if ((zmax >= 0.0) || (zpp >= 0.0))
	{
		std::cerr << "Matrix4x4::perspective: must have zmin < zmax < 0 AND zpp < 0\n";
		return IdentityMatrix;
	}
	return Matrix4x4(
		-2.0*zpp/(xmax - xmin),                    0.0, (xmax + xmin)/(xmax - xmin),                            0.0,
		                   0.0, -2.0*zpp/(ymax - ymin), (ymax + ymin)/(ymax - ymin),                            0.0,
		                   0.0,                    0.0, (zmax + zmin)/(zmax - zmin), -2.0*zmin*zmax / (zmax - zmin),
	                       0.0,                    0.0,                        -1.0,                           0.0);
}

Matrix4x4 Matrix4x4::scale(double sx, double sy, double sz)
{
	Matrix4x4 m;
	m.mElem[0][0] = sx;
	m.mElem[1][1] = sy;
	m.mElem[2][2] = sz;
	return m;
}

void Matrix4x4::setElementAt(int i, int j, double newValue)
{
	if ( (i < 4) && (j < 4) )
		mElem[i][j] = newValue;
}

Matrix3x3 Matrix4x4::subMatrix(int skipRow, int skipCol) const
{
	Matrix3x3	m3x3;
	int R = 0;
	for (int row=0 ; row<4 ; row++)
		if (row != skipRow)
		{
			int C = 0;
			for (int col=0 ; col<4 ; col++)
				if (col != skipCol)
				{
					m3x3.setElementAt(R,C,elementAt(row,col));
					C++;
				}
			R++;
		}
	return m3x3;
}

Matrix4x4 Matrix4x4::translation(const AffVector& translation)
{
	Matrix4x4 M;
	M.mElem[0][3] = translation[DX];
	M.mElem[1][3] = translation[DY];
	M.mElem[2][3] = translation[DZ];
	return M;
}

Matrix4x4 Matrix4x4::xRotationDegrees(double angle)
{
	double radians = degreesToRadians(angle);
	double c = cos(radians), s = sin(radians);
	return Matrix4x4(
		1.0, 0.0, 0.0, 0.0,
		0.0,   c,  -s, 0.0,
		0.0,   s,   c, 0.0,
		0.0, 0.0, 0.0, 1.0
		);
}

Matrix4x4 Matrix4x4::xRotationRadians(double angle)
{
	double c = cos(angle), s = sin(angle);
	return Matrix4x4(
		1.0, 0.0, 0.0, 0.0,
		0.0,   c,  -s, 0.0,
		0.0,   s,   c, 0.0,
		0.0, 0.0, 0.0, 1.0
		);
}

Matrix4x4 Matrix4x4::yRotationDegrees(double angle)
{
	double radians = degreesToRadians(angle);
	double c = cos(radians), s = sin(radians);
	return Matrix4x4(
		  c, 0.0,   s, 0.0,
		0.0, 1.0, 0.0, 0.0,
		 -s, 0.0,   c, 0.0,
		0.0, 0.0, 0.0, 1.0
		);
}

Matrix4x4 Matrix4x4::yRotationRadians(double angle)
{
	double c = cos(angle), s = sin(angle);
	return Matrix4x4(
		  c, 0.0,   s, 0.0,
		0.0, 1.0, 0.0, 0.0,
		 -s, 0.0,   c, 0.0,
		0.0, 0.0, 0.0, 1.0
		);
}

Matrix4x4 Matrix4x4::zRotationDegrees(double angle)
{
	double radians = degreesToRadians(angle);
	double c = cos(radians), s = sin(radians);
	return Matrix4x4(
		  c,  -s, 0.0, 0.0,
		  s,   c, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
		);
}

Matrix4x4 Matrix4x4::zRotationRadians(double angle)
{
	double c = cos(angle), s = sin(angle);
	return Matrix4x4(
		  c,  -s, 0.0, 0.0,
		  s,   c, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
		);
}

} // end namespace
