// Complex.h -- Standard Complex numbers!
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef COMPLEX_H
#define COMPLEX_H

namespace cryph
{

class Complex
{
public:
	Complex(double reIN=0.0, double imIN=0.0);
	Complex(const Complex& c);
	virtual ~Complex();

	double Re() const { return re; }
	double Im() const { return im; }

	double distFromO() const { return ::sqrt(re*re + im*im); }
	double distTo(const Complex& c) const
		{ double d1 = c.re - re; double d2 = c.im - im;
		  return ::sqrt(d1*d1 + d2*d2); }

	// direct solution method:
	int sqrt_v1(Complex root[]) const; // may be up to 2 square roots
	// de Moivre's method:
	int sqrt_v2(Complex root[]) const; // may be up to 2 square roots

	// --------- operators as methods
	Complex		operator=(const Complex& rhs);
	Complex		operator+=(const Complex& rhs);
	Complex		operator-=(const Complex& rhs);
	Complex		operator*=(const Complex& rhs);
	Complex		operator*=(double f);
	Complex		operator/=(const Complex& rhs);
	Complex		operator/=(double f);

	Complex		operator+(const Complex& c) const;
	Complex		operator-(const Complex& c) const;
	Complex		operator*(double f) const;
	Complex		operator*(const Complex& c) const;
	Complex		operator/(const Complex& c) const;

private:
	double	re;
	double	im;
};
	Complex operator*(double f, const Complex& c);

}

#endif
