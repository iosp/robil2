// Complex.c++ -- Standard Complex numbers!
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <iostream>
#include <math.h>

#include <cryph/Complex.h>
#include <cryph/Polynomial.h>

namespace cryph
{

Complex::Complex(double reIN, double imIN) : re(reIN), im(imIN)
{
}

Complex::Complex(const Complex& c) : re(c.re), im(c.im)
{
}

Complex::~Complex()
{
}

Complex Complex::operator=(const Complex& rhs)
{
	re = rhs.re; im = rhs.im;
	return *this;
}

Complex Complex::operator+=(const Complex& rhs)
{
	re += rhs.re;
	im += rhs.im;
	return *this;
}

Complex Complex::operator-=(const Complex& rhs)
{
	re -= rhs.re;
	im -= rhs.im;
	return *this;
}

Complex Complex::operator*=(const Complex& rhs)
{
	double newRE = re*rhs.re - im*rhs.im;
	double newIM = re*rhs.im + im*rhs.re;
	re = newRE; im = newIM;
	return *this;
}

Complex Complex::operator*=(double f)
{
	re *= f;
	im *= f;
	return *this;
}

Complex Complex::operator/=(const Complex& rhs)
{
	double denom = rhs.re*rhs.re + rhs.im*rhs.im;
	double newRE = (re*rhs.re + im*rhs.im) / denom;
	double newIM = (im*rhs.re - re*rhs.im) / denom;
	re = newRE; im = newIM;
	return *this;
}

Complex Complex::operator/=(double f)
{
	re /= f; im /= f;
	return *this;
}

Complex Complex::operator+(const Complex& c) const
{
	return Complex(re+c.re, im+c.im);
}

Complex Complex::operator-(const Complex& c) const
{
	return Complex(re-c.re, im-c.im);
}

Complex Complex::operator*(double f) const
{
	return Complex(f*re, f*im);
}

Complex Complex::operator*(const Complex& c) const
{
	return Complex(re*c.re - im*c.im, re*c.im + im*c.re);
}

Complex Complex::operator/(const Complex& c) const
{
	double denom = c.re*c.re + c.im*c.im;
	return Complex(
		(re*c.re + im*c.im)/denom,
		(im*c.re - re*c.im)/denom);
}

// external functions

Complex operator*(double f, const Complex& c)
{
	return Complex(f*c.Re(), f*c.Im());
}

// sqrt:
// 1. Given this: (c,d); i.e., c + d*i
// 2. Compute the sqrt(this): (a,b); i.e., a + b*i
// Method
// if d==0, then simple. Either:
//      return (sqrt(c), 0)  // if c >= 0
//      return (0, sqrt(-c)) // if c < 0
// else result will be Complex (b != 0)
// sqrt(c + d*i) = a + b*i
// Square both sides and equate real and imaginary terms:
// c = a*a - b*b; d = 2*a*b
// From "d" equation: a = d/(2*b). Substitute into "c" eqn & solve for "b^2"
// 4b^4 + 4(b^2)c - d^2 = 0. Use quadratic formula to solve for b^2.
// This yields two values for b^2> Use the square root of the positive value
// to get b and hence a.
// "root" is assumed to be at least 2 positions long; return value is number of
// unique roots computed.
int Complex::sqrt_v1(Complex root[]) const
{
	// match terminology above:
	double c = re, d = im;

	// See if the given number is really real:
	if (fabs(d) < 1.0e-6)
	{
		// Given number is real
		if (fabs(c) <= 1.0e-6)
		{
			root[0].re = 0.0;
			root[0].im = 0.0;
			return 1;
		}
		if (c >= 0)
		{
			root[1].re = -(root[0].re = ::sqrt(c));
			root[0].im = root[1].im = 0.0;
		}
		else
		{
			root[1].im = -(root[0].im = ::sqrt(-c));
			root[0].re = root[1].re = 0.0;
		}
		return 2;
	}

	// Given number is complex ==> roots are complex:
	// Solve for b^2:
	double coeff[3] = {
		-d*d, 4.0*c, 4.0
	};
	Polynomial bSqrPoly(coeff, 2);
	Polynomial::Root bSqrRoots[2];
	int n_bSqrRoots = bSqrPoly.solve(bSqrRoots);

	int nComplexRoots = 0;
	for (int i=0 ; i<n_bSqrRoots ; i++)
	{
		if (bSqrRoots[i].value >= 0.0)
		{
			if (nComplexRoots > 0)
			{
				std::cerr << "Complex::sqrt_v1 found too many square roots!\n";
				return nComplexRoots;
			}
			// We get two roots, one with +b and one with -b:
			double b = ::sqrt(bSqrRoots[i].value);
			double a = d / (2.0*b);
			root[nComplexRoots].re = a;
			root[nComplexRoots++].im = b;
			root[nComplexRoots].re = -a;
			root[nComplexRoots++].im = -b;
		}
	}
	return nComplexRoots;
}

int Complex::sqrt_v2(Complex root[]) const
{
	// de Moivre's method
	double r = ::sqrt(re*re + im*im);
	double theta = atan2(im, re);
	double rootR = ::sqrt(r);
	double cosThBy2 = cos(theta/2.0);
	double sinThBy2 = sin(theta/2.0);
	root[1].re = -(root[0].re = rootR*cosThBy2);
	root[1].im = -(root[0].im = rootR*sinThBy2);
	return 2;
}

} // end namespace cryph
