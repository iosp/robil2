// Matrix4x4.h -- 4x4 Matrices
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

// See comments at start of Matrix3x3.h

#ifndef MATRIX4x4_H
#define MATRIX4x4_H

#include <iostream>

#include "AffPoint.h"
#include "AffVector.h"
#include "Matrix3x3.h"

namespace cryph
{

class ProjPoint;
class ViewVolume;

class Matrix4x4
{
public:
	Matrix4x4(); // Identity Matrix
	Matrix4x4(const Matrix4x4& m);
	Matrix4x4(
		double m11, double m12, double m13, double m14,
		double m21, double m22, double m23, double m24,
		double m31, double m32, double m33, double m34,
		double m41, double m42, double m43, double m44);

	// From 3x3
	// ** This is (currently) the only interface in either Matrix3x3 or
	// ** Matrix4x4 that supports 2D transformations in any way.
	// This assumes "M" describes a 2D xform in projective space.
	// The implementation slides the third row and third column over
	// to the fourth, and inserts a third row and third column from
	// a 4x4 Identity matrix.
	Matrix4x4(const Matrix3x3& M);

	// From an Affine transformation spec: (M, t). 4th row gets (0,0,0,1)
	Matrix4x4(const Matrix3x3& M, const AffVector& t);

	// From an affine matrix M and a fixed point:
	Matrix4x4(const Matrix3x3& M, const AffPoint& FixedPoint);

	// From an affine matrix M and a point whose pre- and post-image are known
	Matrix4x4(const Matrix3x3& M,
			const AffPoint& PreImage, const AffPoint& PostImage);

	virtual ~Matrix4x4();

	// class "Factory" methods to create 4x4 matrices:
	// 1. from arrays of length 16:
	static Matrix4x4 fromColMajor(const float* m);  // 'm' assumed to be [16]
	static Matrix4x4 fromColMajor(const double* m); // 'm' assumed to be [16]
	static Matrix4x4 fromRowMajor(const float* m);  // 'm' assumed to be [16]
	static Matrix4x4 fromRowMajor(const double* m); // 'm' assumed to be [16]
	// 2. Rotation affine transformations:
	static Matrix4x4 xRotationDegrees(double angle);
	static Matrix4x4 yRotationDegrees(double angle);
	static Matrix4x4 zRotationDegrees(double angle);
	static Matrix4x4 xRotationRadians(double angle);
	static Matrix4x4 yRotationRadians(double angle);
	static Matrix4x4 zRotationRadians(double angle);
	// 3. Translation affine transformation
	static Matrix4x4 translation(const AffVector& trans);
	// 4. Scale affine transformation
	static Matrix4x4 scale(double sx, double sy, double sz);
	// 5. 3D Viewing interfaces
	static Matrix4x4 lookAt(const cryph::AffPoint& eye,
							const cryph::AffPoint& center,
							const cryph::AffVector& up);
	static Matrix4x4 orthogonal(double xmin, double xmax, double ymin, double ymax,
				double zmin, double zmax);
	static Matrix4x4 oblique(double zpp, double xmin, double xmax, double ymin,
				double ymax, double zmin, double zmax,
				const cryph::AffVector& projDir);
	static Matrix4x4 perspective(double zpp, double xmin, double xmax,
			double ymin, double ymax, double zmin, double zmax);

	// A class method routine primarily designed for internal use while
	// implementing lookAt, but useful by clients if they want to
	// ensure that a given set of parameters are valid. (The bool
	// return is true if and only if the output "v" and "w" parameters
	// have been correctly computed on return.)
	static bool getECvw(const cryph::AffPoint& eye,
			const cryph::AffPoint& center, const cryph::AffVector& up,
			cryph::AffVector& v, cryph::AffVector& w);
	// END: class "Factory" methods to create 4x4 matrices

	// Instance methods

	Matrix4x4 operator=(const Matrix4x4& rhs);

	Matrix4x4 operator*=(const Matrix4x4& rhs);
	Matrix4x4 operator*=(double f);
	Matrix4x4 operator+=(const Matrix4x4& rhs);

	AffPoint operator*(const AffPoint& p) const;
	ProjPoint operator*(const ProjPoint& p) const;
	AffVector operator*(const AffVector& v) const;

	Matrix4x4 operator*(const Matrix4x4& m2) const;
	Matrix4x4 operator+(const Matrix4x4& m2) const;
	Matrix4x4 operator-(const Matrix4x4& m2) const;

	friend Matrix4x4 operator*(double f, const Matrix4x4& m);
	friend std::ostream& operator<<(std::ostream& os, const Matrix4x4& m);
	friend std::istream& operator>>(std::istream& is, Matrix4x4& m);

	double determinant() const;
	double elementAt(int i, int j) const;

	// Extraction routines useful in OpenGL, OpenInventor, etc.
	float* extractColMajor(float m[16]) const;
	double* extractColMajor(double m[16]) const;
	float* extractRowMajor(float m[16]) const;
	double* extractRowMajor(double m[16]) const;

	// extractAffineMt simply places the upper left 3x3 into M, and
	// the first three rows of the fourth column into t. It does not
	// check to make sure that the 4th row is (0, 0, 0, 1). (First use
	// "isAffineTransformation" if this might be an issue.)
	void extractAffineMt(Matrix3x3& M, AffVector& t) const;

	// The following routine uses Matrix3x3::extractAxisAngle to
	// find the unit axis vector and angle to reconstruct the upper
	// 3x3 portion of the matrix. It then computes a base point B
	// which, when combined with (w,theta) will at least
	// approximate the entire 4x4 matrix. At worst, a residual
	// "postTranslation" will be required to reproduce the effect
	// of the original 4x4 matrix. That translation (possibly 0)
	// is returned in "postTranslation".
	int extractAxisAngle(AffPoint& B, AffVector& w, double& theta,
					AffVector& postTranslation) const;
	// Following routine NOT highly numerical stable. It is OK with
	// friendly matrices. For a better matrix inverse implementation for
	// when the matrix may be poorly conditioned, use MatrixNxN::inverse.
	bool inverse(Matrix4x4& mInv) const;
	// isAffineTransformation returns an int: bit 0 set iff M44==1;
	// bit 1 set iff (M41, M42, M43) != (0, 0, 0). Hence, for example,
	// if the 4th row is (0, 0, 0, 1), the return value will be 1.
	int isAffineTransformation() const;
	// In the following two multiply routines, nElements describes the
	// length of the 'a' and 'b' arrays. If nElements<4, then (i) a[3]
	// is assumed to be 1.0, and (ii) a[i] for nElements<=i<3 are
	// assumed to be zero. nElements cannot be larger than 4.
	// Only nElements positions of 'b' will be set.
	void multiply(const double a[], double b[], int nElements=4) const;
	void multiply(const float a[], float b[], int nElements=4) const;

	void setElementAt(int i, int j, double newValue);
	Matrix3x3 subMatrix(int skipRow, int skipCol) const;


    // ---------- Global constants

	// 1. Special Matrices
	static const Matrix4x4 IdentityMatrix;
	static const Matrix4x4 ZeroMatrix;

	// 2. Return codes for extraction of base point, unit axis vector,
	//    and rotation angle from 4x4 matrices that are 'supposed to
	//    describe' affine transformations whose upper 3x3 matrix is
	//    'supposed to be' orthogonal and right-handed. (See the Matrix3x3
	//    method "extractAxisAngle".)

	// 2.1: Abnormal internal errors that should never be returned:
	static const int InternalBasePointComputationError;

	// 2.2: "Normal" errors if user provides inappropriate matrix:
	static const int NotAffine;
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
	static const int Extracted_BwTheta;

protected:
	double mElem[4][4];

private:
	void copy(const Matrix4x4& rhs);

	static int determineBasePoint(AffVector ImMRows[], int maxWCompLoc,
					int r1, int r2, const AffVector& trans, AffPoint& B);

	void installMt(const Matrix3x3& M, const AffVector& t);
};

}

#endif
