// Formatter.h -- class that manages io specifications for points, vectors,
//             matrices, and the like.
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef FORMATTER_H
#define FORMATTER_H

#include <iostream>
#include <string>
#include "AffPoint.h"
#include "AffVector.h"
#include "Matrix3x3.h"
#include "Matrix4x4.h"
#include "MatrixMxN.h"
#include "ProjPoint.h"

namespace cryph
{

class Formatter
{
public:
	Formatter(
		std::string start="(", std::string end=")", std::string sep=", ",
		std::string rowStart="(", std::string rowEnd=")", std::string rowSep=", ");
	~Formatter();

	// Following applicable to all supported types
	void setStartEndStrings(std::string start, std::string end);
	void setElementSeparator(std::string sep);

	// Following only applicable to matrix types
	void setRowStartEndStrings(std::string start, std::string end);
	void setRowSeparator(std::string sep);

	void fromStream(std::istream& is, AffPoint& p) const;
	void fromStream(std::istream& is, AffVector& v) const;
	void fromStream(std::istream& is, Matrix3x3& m) const;
	void fromStream(std::istream& is, Matrix4x4& m) const;
	void fromStream(std::istream& is, MatrixMxN& m) const;
	void fromStream(std::istream& is, ProjPoint& p) const;

	std::string toString(double v) const;
	std::string toString(const AffPoint& p) const;
	std::string toString(const AffVector& v) const;
	std::string toString(const Matrix3x3& m) const;
	std::string toString(const Matrix4x4& m) const;
	std::string toString(const MatrixMxN& m) const;
	std::string toString(const ProjPoint& p) const;

private:
	std::string startString, endString, elementSeparator;
	std::string rowStartString, rowEndString, rowSeparator;
	// Following needed when reading from a stream assumed to have been
	// built by this (or an equivalent) Formatter:
	int numNonWhiteStartChars, numNonWhiteElementSeparatorChars,
	    numNonWhiteEndChars;
	int numNonWhiteRowStartChars, numNonWhiteRowSeparatorChars,
	    numNonWhiteRowEndChars;

	static int countNumNonWhite(std::string str);
};

}

#endif
