// Polynomial.h -- General degree n polynomial
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

namespace cryph
{

class Polynomial
{
public:
	Polynomial(double coeff[], int degree);
	Polynomial(const Polynomial& p);
	virtual ~Polynomial();

	struct Root
	{
		double	value;
		int		multiplicity;
	};

	// function return value is the polynomial evaluated at t
	double	evaluate(double t) const;
	double	evaluate(double t, double& fPrime) const;
	// function return value is number of roots located
	// the given Root array assumed to be sufficiently large to hold all roots
	int		solve(Polynomial::Root ans[]);

private:
	double*	mCoeff;
	int		mDegree;

	double	mCoeffZero, mFcnZero; // only used during equation solving...

	int		addZeroRoots(Polynomial::Root ans[],
				int nZeros, int nTrailingZeroCoefficients) const;
	double	findBoundedRoot(double c[], int deg, double v1, double v2) const;
	double	findRoot(double c[], int deg, double guess) const;
	int		removeTrailingZeros(double c[], int deg) const;
	int		solveNormalizedPoly(double coeff[],int deg, Polynomial::Root ans[]);
	int		solveQuadratic(double a,double b,double c, Polynomial::Root ans[]);

	static double evaluate(const double a[], int deg, double t);
	static double evaluate(const double a[], int deg, double t, double& fPrime);
};

}

#endif
