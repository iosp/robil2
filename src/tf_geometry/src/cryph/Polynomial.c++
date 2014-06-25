// Polynomial.c++ -- General degree n polynomial
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <iostream>
using namespace std;
#include <stdlib.h>
#include <math.h>

#include <cryph/Polynomial.h>
#include <cryph/Inline.h>

#define COMPILE_WITH_TEST_MAIN_PROGRAM 0

#define COEFF_ZERO              1.0E-6
#define DISC_ZERO               1.0E-8
#define FCN_ZERO                1.0E-13
#define MAX_DEGREE              30
#define MAX_ITERATIONS          200
#define MISC_TOL                1.0E-5
#define NEWTON_CONVERGENCE      1.0E-12

namespace cryph
{

Polynomial::Polynomial(double coeff[], int degree) :
	mCoeff(NULL), mDegree(degree)
{
	if (mDegree > 0)
	{
		mCoeff = new double[mDegree+1];
		for (int i=0 ; i<=mDegree ; i++)
			mCoeff[i] = coeff[i];
	}
}

Polynomial::Polynomial(const Polynomial& p) :
	mCoeff(NULL), mDegree(p.mDegree)
{
	if (mDegree > 0)
	{
		mCoeff = new double[mDegree+1];
		for (int i=0 ; i<=mDegree ; i++)
			mCoeff[i] = p.mCoeff[i];
	}
}

Polynomial::~Polynomial()
{
	if (mCoeff != NULL)
	{
		delete [] mCoeff;
		mCoeff = NULL;
	}
}

int Polynomial::addZeroRoots(Polynomial::Root ans[],
		int nZeros, int nTrailingZeroCoefficients) const
{
	/* The polynomial whose roots are stored in "ans" was originally
	presented as a polynomial with "nTrailingZeroCoefficients" trailing
	zero coefficients. This routine inserts "0" as a root of multiplicity
	"nTrailingZeroCoefficients". */

	if (nTrailingZeroCoefficients == 0)
		return(nZeros);

	/* There was at least one trailing zero. If nZeros is negative, then
	it needs to be reset to zero. (Negative meant that no roots were found
	for one reason or another. There is now clearly at least one root.) */

	if (nZeros < 0)
		nZeros = 0;

	/* add the zero at the appropriate spot in the ordered list of roots */

	int i=nZeros-1;
	for ( ; (i>=0) && (ans[i].value>0.0) ; i--)
		ans[i+1] = ans[i];

	/* the zero root needs to go in at ans[i+1] */

	ans[++i].value = 0.0;
	ans[i].multiplicity = nTrailingZeroCoefficients;

	return (nZeros+1);
}

double Polynomial::evaluate(double t) const
{
	return Polynomial::evaluate(mCoeff,mDegree,t);
}

double Polynomial::evaluate(double t, double& fPrime) const
{
	return Polynomial::evaluate(mCoeff,mDegree,t,fPrime);
}

double Polynomial::evaluate( // CLASS METHOD
	const double a[],	/* coefficient array; a[i] is coeff for t**i */
	int          n,		/* degree of polynomial */
	double       t,
	double&      fPrime)
{
	double f = 0.0;
	fPrime   = 0.0;
	for (int i=n ; i>=1 ; i--)
	{
		double coeff   = a[i];
		f              = t*f + coeff;
		fPrime = t*fPrime + (double) i * coeff;
	}
	return t*f + a[0];
}

double Polynomial::evaluate( // CLASS METHOD
	const double a[],	/* coefficient array; a[i] is coeff for t**i */
	int          n,		/* degree of polynomial */
	double       t)
{
	double fPrime;
	return evaluate(a,n,t,fPrime);
}

double Polynomial::findBoundedRoot(
	double	c[],
	int		deg,
	double	v1,
	double	v2
		) const
{
	/* There is a root between v1 and v2. Keep calling findRoot with a
	judiciously chosen start point in the interval until the root is found.
	The start point will initially be the midpoint of the interval. If that
	fails, then we try 1/4 and 3/4 across the interval, and so on. */

	bool	haveRoot = false;
	double	base = 0.5, inc = 1.0;

	double v = 0.0;
	for ( ; !haveRoot ; base*=0.5, inc*=0.5)
		for (double f=base ; f<1.0 ; f+=inc)
		{
			double t = v1 + f*(v2 - v1);
			v = findRoot(c,deg,t);
			if ((v > v1) && (v < v2))
			{
				haveRoot = true;
				break;
			}
		}

	return v;
}

double Polynomial::findRoot(
	double	c[],
	int		deg,
	double	guess
		) const
{
	// initialize tolerances
	double cur_guess = guess;
	double newton_convergence;
	if (fabs(cur_guess) < MISC_TOL)
		newton_convergence = NEWTON_CONVERGENCE;
	else
		newton_convergence = NEWTON_CONVERGENCE * fabs(cur_guess);

	int nIterations = 0;
	double last_guess = cur_guess, fPrime;
	do
	{
		last_guess = cur_guess;
		double f = evaluate(c,deg,last_guess,fPrime);
		cur_guess = last_guess - (f/fPrime);
	} while ( (fabs(last_guess - cur_guess) > newton_convergence) &&
		  (++nIterations < MAX_ITERATIONS) );

	return cur_guess;
}

int Polynomial::removeTrailingZeros(double c[], int deg) const
{
	/* lower the degree of the given polynomial by dividing by t^n where
	n is the number of trailing zeros. (i.e., c[i]=0, i=0,(n-1).) Return
	the number of trailing zeros found. This routine assumes that the
	polynomial is not the zero polynomial. */

	int n = 0;
	while (fabs(c[n]) <= mCoeffZero)
		n++;
	if (n != 0)
		for (int i=0, j=n ; j<=deg ; )
			c[i++] = c[j++];
	return n;
}

int Polynomial::solve(Polynomial::Root ans[])
{
	double maxAbsCoeff = fabs(mCoeff[mDegree]);
	for (int i=0 ; i<mDegree ; i++)
		if (fabs(mCoeff[i]) > maxAbsCoeff)
			maxAbsCoeff = fabs(mCoeff[i]);
	mCoeffZero = COEFF_ZERO * maxAbsCoeff;

	int deg = mDegree;
	while ( (deg > 0) && (fabs(mCoeff[deg]) < mCoeffZero) )
		deg--;

	if (deg <= 0)
		return -1;

	// We impose a maximum degree in what is probably a misplaced concern
	// related to efficiency. The method "solveNormalizedPoly" is recursive,
	// and it has to allocate a coefficient array and a Root array on every
	// call. By imposing a maximum degree, we can avoid lots of dynamic memory
	// allocation and deallocation.
	if (deg > MAX_DEGREE)
		return -3;

	/* normalize the polynomial and call the recursive root finder */

	double t = 1.0 / mCoeff[deg];
	maxAbsCoeff = 1.0;
	double	coeff[MAX_DEGREE+1];
	for (int i=0 ; i<deg ; i++)
	{
		double a = coeff[i] = mCoeff[i] * t;
		if (fabs(a) > maxAbsCoeff)
			maxAbsCoeff = fabs(a);
	}
	coeff[deg] = 1.0;
	mCoeffZero = COEFF_ZERO * maxAbsCoeff;
	mFcnZero   = FCN_ZERO * maxAbsCoeff;

	int n = solveNormalizedPoly(coeff,deg,ans);
	return n;
}

int Polynomial::solveNormalizedPoly(
	double			c[],	/* coefficient array; c[i] is coeff for t**i */
	int				deg,
	Polynomial::Root	ans[]
		)
{
	/* Entry conditions:
				deg > 0
				c[deg] = 1.0
				mCoeffZero and mFcnZero are initialized with values
					appropriate for the input polynomial */

	Polynomial::Root	answer[MAX_DEGREE];
	double			coeff[MAX_DEGREE+1], roots[2], f, fp, ff, delta, t;
	double			maxAbsCoeff, save_cz, save_fz;

	int nTrailingZeros = removeTrailingZeros(c,deg);
	deg -= nTrailingZeros;

	int n_zeros = 0;
	if (deg <= 2)
	{
		double a = 0.0;
		if (deg == 2)
			a = c[2];
		n_zeros = solveQuadratic(a,c[1],c[0],ans);
	}
	else /* find zeros of the derivative and use those to bound fcn roots */
	{
		/* form derivative polynomial with leading coefficient=1.0 */

		t = 1.0 / ((double) deg);
		maxAbsCoeff = 1.0;
		for (int i=1 ; i<deg ; i++)
		{
			double a = coeff[i-1] = (double) i * c[i] * t;
			if (fabs(a) > maxAbsCoeff)
				maxAbsCoeff = fabs(a);
		}
		coeff[deg-1] = 1.0;

		save_cz = mCoeffZero; mCoeffZero = COEFF_ZERO * maxAbsCoeff;
		save_fz = mFcnZero; mFcnZero = FCN_ZERO * maxAbsCoeff;
		int n = solveNormalizedPoly(coeff,(deg-1),answer);
		mCoeffZero = save_cz;
		mFcnZero   = save_fz;

		if (n <= 0) /* derivative never zero ==> fcn is monotonic */
		{
			ans[0].value = findRoot(c,deg,0.0);
			ans[0].multiplicity = 1;
			return 1;
		}
		double f = evaluate(c,deg,answer[0].value);
		bool prev_was_zero;
		if (fabs(f) < mFcnZero) /* function and derivative BOTH zero */
		{
			prev_was_zero = true;
			ans[0] = answer[0];
			ans[0].multiplicity++;
			n_zeros = 1;
		}
		else /* only the derivative is zero here */
		{
			prev_was_zero = false;
			if (fabs(answer[0].value) > 1.0)
			{
				delta = 0.1 * answer[0].value;
				if (delta > 0.0) delta = -delta;
			}
			else delta = -2.0;
			t = answer[0].value + delta;
			double ff = evaluate(c,deg,t);
			if ( ((f>0.0) && (ff<f)) ||
			     ((f<0.0) && (ff>f)) )
			{
				int nIterations = 0;
				do
				{
					ans[0].value = findRoot(c,deg,t);
					t += delta;
				} while ( (ans[0].value > answer[0].value) &&
					  (++nIterations < MAX_ITERATIONS) );
				ans[0].multiplicity = 1;
				n_zeros = 1;
			}
			else
				n_zeros = 0;
		}
		for (int i=1 ; i<n ; i++)
		{
			double ff = evaluate(c,deg,answer[i].value);
			if (prev_was_zero)
				prev_was_zero = false;
			else if (fabs(ff) < mFcnZero) /* fcn & deriv zero */
			{
				prev_was_zero = true;
				ans[n_zeros] = answer[i];
				ans[n_zeros].multiplicity++;
				n_zeros++;
			}
			else if ( ((f>0.0) && (ff<0.0)) ||
				  ((f<0.0) && (ff>0.0)) )
			{
				/* there is a root in this interval */
				ans[n_zeros].value = findBoundedRoot(c,deg,
					answer[i-1].value,answer[i].value);
				ans[n_zeros].multiplicity = 1;
				n_zeros++;
			}
			f = ff;
		}
		if (!prev_was_zero)
		{
			if (fabs(answer[n-1].value) > 1.0)
			{
				delta = 0.1 * answer[n-1].value;
				if (delta < 0.0) delta = -delta;
			}
			else delta = 2.0;
			t = answer[n-1].value + delta;
			double ff = evaluate(c,deg,t);
			if ( ((f>0.0) && (ff<f)) || ((f<0.0) && (ff>f)) )
			{
				int nIterations = 0;
				do
				{
					ans[n_zeros].value = findRoot(c,deg,t);
					t += delta;
				} while ( (ans[n_zeros].value < answer[n-1].value) &&
					  (++nIterations < MAX_ITERATIONS) );
				ans[n_zeros].multiplicity = 1;
				n_zeros++;
			}
		}
	}
	n_zeros = addZeroRoots(ans,n_zeros,nTrailingZeros);
	return n_zeros;
}

int Polynomial::solveQuadratic(double a, double b, double c,
			Polynomial::Root roots[2])
{
	double maxAbsCoeff = fabs(a);
	if (fabs(b) > maxAbsCoeff)
		maxAbsCoeff = fabs(b);
	if (fabs(c) > maxAbsCoeff)
		maxAbsCoeff = fabs(c);
	mCoeffZero = COEFF_ZERO * maxAbsCoeff;
	double disc_zero  = DISC_ZERO * maxAbsCoeff;

	if (fabs(a) < mCoeffZero)
		if (fabs(b) < mCoeffZero)
			return -1;
		else
		{
			roots[0].value = -c / b;
			roots[0].multiplicity = 1;
			return 1;
		}
	double inv_denom = 0.5 / a;
	double disc = b*b - 4.0*a*c;
	if (fabs(disc) < disc_zero)
	{
		roots[0].value = -b * inv_denom;
		roots[0].multiplicity = 2;
		return 1;
	}
	if (disc < 0.0)
		return -2;
	disc = sqrt(disc);
	roots[0].value = (-b - disc) * inv_denom;
	roots[1].value = (-b + disc) * inv_denom;
	if (roots[0].value > roots[1].value)
		swap2(roots[0],roots[1]);
	roots[0].multiplicity = roots[1].multiplicity = 1;
	return 2;
}

}

#if COMPILE_WITH_TEST_MAIN_PROGRAM

using namespace cryph;

int main()
{
	cout << "Enter degree: ";
	int deg; cin >> deg;
	double coeff[30];
	Polynomial::Root roots[30];
	while (deg > 0)
	{
		for (int i=deg ; i>= 0 ; i--)
		{
			cout << "coeff[" << i << "] = ";
			cin >> coeff[i];
		}
		Polynomial p(coeff,deg);
		int nRoots = p.solve(roots);
		cout << "Found " << nRoots << " roots.\n";
		for (int i=0 ; i<nRoots ; i++)
		{
			double fOfT = p.evaluate(roots[i].value);
			cout << "Root " << i << " = " << roots[i].value << " (mult = "
			     << roots[i].multiplicity << "), f(root) = " << fOfT << '\n';
		}
		cout << "\n\nEnter degree: ";
		cin >> deg;
	}
	return 0;
}

#endif
