// Matrix3x3.h -- 3x3 Matrices
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

// This interface is designed for the specification and manipulation of
// 3x3 matrices as a part of Affine transformations specified on 3D
// vectors. As such, it provides facilities to create 3x3 matrices that
// rotate and mirror vectors, form tensor products of vectors, and provide
// other support required by Matrix4x4 in its role of representing
// transformations of 3D geometry. This interface is NOT intended to
// provide facilities for representing 2D transformations in homogeneous
// form.

#ifndef MATRIX3x3_H
#define MATRIX3x3_H

#include "AffPoint.h"
#include "AffVector.h"

namespace cryph
{

class Matrix3x3
{
public:
	Matrix3x3(); // Identity Matrix
	Matrix3x3(const Matrix3x3& M);
	Matrix3x3(
		double m11, double m12, double m13,
		double m21, double m22, double m23,
		double m31, double m32, double m33);
	virtual ~Matrix3x3();

	// class "Factory" methods to create 3x3 matrices:
	// 1. Rotation affine transformations
	static Matrix3x3 xRotationDegrees(double angle);
	static Matrix3x3 xRotationRadians(double angle);
	static Matrix3x3 yRotationDegrees(double angle);
	static Matrix3x3 yRotationRadians(double angle);
	static Matrix3x3 zRotationDegrees(double angle);
	static Matrix3x3 zRotationRadians(double angle);
	static Matrix3x3 generalRotationRadians
					(const AffVector& rotationAxis, double angle);
	// 2. Align one or two pairs of vectors
	static Matrix3x3 alignVectors
					(const AffVector& uFrom, const AffVector& uTo);
	static Matrix3x3 alignVectors
					(const AffVector& uFrom, const AffVector& vFrom,
					 const AffVector& uTo,   const AffVector& vTo);
	// 3. Scale and shear
	static Matrix3x3 scale
					(double sx, double sy, double sz);
	static Matrix3x3 shear
					(const AffVector& n, const AffVector& u, double f);
	// 4. Miscellaneous
	static Matrix3x3 crossProductMatrix(const AffVector& u);
	static Matrix3x3 mirrorMatrix(const AffVector& mirrorPlaneNormal);
	static Matrix3x3 tensorProductMatrix
							(const AffVector& u, const AffVector& v);

	// Instance methods

	Matrix3x3 operator=(const Matrix3x3& rhs);

	Matrix3x3 operator*=(const Matrix3x3& rhs);
	Matrix3x3 operator*=(double f);
	Matrix3x3 operator+=(const Matrix3x3& rhs);

	AffPoint operator*(const AffPoint& p) const;
	AffVector operator*(const AffVector& v) const;

	Matrix3x3 operator*(const Matrix3x3& m2) const;
	Matrix3x3 operator+(const Matrix3x3& m2) const;
	Matrix3x3 operator-(const Matrix3x3& m2) const;
	
	friend Matrix3x3 operator*(double f, const Matrix3x3& m);
	friend std::ostream& operator<<(std::ostream& os, const Matrix3x3& m);
	friend std::istream& operator>>(std::istream& is, Matrix3x3& m);

	double determinant() const;
	int eigenValues(double lambda[], AffVector eigenV[], int* mult=NULL)
					const;
	double elementAt(int i, int j) const;
	float* extractColMajor(float m[9]) const;
	double* extractColMajor(double m[9]) const;
	float* extractRowMajor(float m[9]) const;
	double* extractRowMajor(double m[9]) const;
	int extractAxisAngle(AffVector& w, double& theta) const;
	void extractRows(AffVector& row1, AffVector& row2, AffVector& row3) const;
	bool inverse(Matrix3x3& mInv) const;
	bool isOrthogonal() const;
	bool isRightHanded() const;
	double largestDiagonalElement(int& pos) const;
	// in the following, 'a' and 'b' are asssumed to be arrays[3]
	void multiply(const double a[], double b[]) const;
	void setElementAt(int i, int j, double newValue);
	double trace() const;
	void transpose();

    // ---------- Global constants

	// 1. Special Matrices
	static const Matrix3x3 IdentityMatrix;
	static const Matrix3x3 ZeroMatrix;

	// 2. Return codes for extraction of unit axis vector & rotation angle
	//    from 3x3 matrices that are 'supposed to be' orthogonal and
	//    right-handed.

	// 2.1: Abnormal internal errors that should never be returned:
	static const int ImMDeterminantNotZero;
	static const int CannotDetermineUnitAxisVector;
	static const int CosTermsNotEqual;
	static const int SinTermsNotEqual;

	// 2.2: "Normal" errors if user provides inappropriate matrix:
	static const int NotOrthogonal;
	static const int NotRightHanded;

	// 3.3: Successful extraction of unit axis vector and angle:
	static const int Extracted_wTheta;

	friend class Matrix4x4;

protected:
	double	mElem[3][3];

private:
	void copy(const Matrix3x3& rhs);
	int computeImMRows(Matrix3x3& ImM,
						// the rows of I-M and the indices of the
						// two most linearly independent rows.
						AffVector rows[], int& r1, int& r2) const;
	int extractPrimitiveAxisAngle(int pos, AffVector& w, double& theta) const;
};

}

#endif
