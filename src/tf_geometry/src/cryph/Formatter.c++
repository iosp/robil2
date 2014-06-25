// Formatter.c++ -- implementation of class that manages io specifications for
//               points, vectors, matrices and the like.
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#include <ctype.h>
#include <string.h>
#include <sstream>

#include <cryph/Formatter.h>
#include <cryph/Inline.h>

namespace cryph
{

Formatter::Formatter(std::string start, std::string end, std::string sep,
	std::string rowStart, std::string rowEnd, std::string rowSep)
{
	setStartEndStrings(start, end);
	setElementSeparator(sep);
	setRowStartEndStrings(rowStart, rowEnd);
	setRowSeparator(rowSep);
}

Formatter::~Formatter()
{
}

int Formatter::countNumNonWhite(std::string str)
{
	int c = 0;
	for (int i=0 ; i<str.length() ; i++)
		if (!isspace(str[i]))
			c++;
	return c;
}

void Formatter::fromStream(std::istream& is, AffPoint& p) const
{
	skipNonblankChars(is, numNonWhiteStartChars);
	is >> p.x;
	skipNonblankChars(is, numNonWhiteElementSeparatorChars);
	is >> p.y;
	skipNonblankChars(is, numNonWhiteElementSeparatorChars);
	is >> p.z;
	skipNonblankChars(is, numNonWhiteEndChars);
}

void Formatter::fromStream(std::istream& is, AffVector& v) const
{
	skipNonblankChars(is, numNonWhiteStartChars);
	is >> v.dx;
	skipNonblankChars(is, numNonWhiteElementSeparatorChars);
	is >> v.dy;
	skipNonblankChars(is, numNonWhiteElementSeparatorChars);
	is >> v.dz;
	skipNonblankChars(is, numNonWhiteEndChars);
}

void Formatter::fromStream(std::istream& is, Matrix3x3& m) const
{
	skipNonblankChars(is, numNonWhiteStartChars);
	double v;
	for (int i=0 ; i<3 ; i++)
	{
		if (i > 0)
			skipNonblankChars(is, numNonWhiteRowSeparatorChars);
		skipNonblankChars(is, numNonWhiteRowStartChars);
		for (int j=0 ; j<3 ; j++)
		{
			if (j > 0)
				skipNonblankChars(is, numNonWhiteElementSeparatorChars);
			is >> v;
			m.setElementAt(i,j, v);
		}
		skipNonblankChars(is, numNonWhiteRowEndChars);
	}
	skipNonblankChars(is, numNonWhiteEndChars);
}

void Formatter::fromStream(std::istream& is, Matrix4x4& m) const
{
	skipNonblankChars(is, numNonWhiteStartChars);
	double v;
	for (int i=0 ; i<4 ; i++)
	{
		if (i > 0)
			skipNonblankChars(is, numNonWhiteRowSeparatorChars);
		skipNonblankChars(is, numNonWhiteRowStartChars);
		for (int j=0 ; j<4 ; j++)
		{
			if (j > 0)
				skipNonblankChars(is, numNonWhiteElementSeparatorChars);
			is >> v;
			m.setElementAt(i,j, v);
		}
		skipNonblankChars(is, numNonWhiteRowEndChars);
	}
	skipNonblankChars(is, numNonWhiteEndChars);
}

void Formatter::fromStream(std::istream& is, MatrixMxN& m) const
{
	int nr = m.getNumRows();
	int nc = m.getNumCols();
	skipNonblankChars(is, numNonWhiteStartChars);
	double v;
	for (int i=0 ; i<nr ; i++)
	{
		if (i > 0)
			skipNonblankChars(is, numNonWhiteRowSeparatorChars);
		skipNonblankChars(is, numNonWhiteRowStartChars);
		for (int j=0 ; j<nc ; j++)
		{
			if (j > 0)
				skipNonblankChars(is, numNonWhiteElementSeparatorChars);
			is >> v;
			m.setElementAt(i,j, v);
		}
		skipNonblankChars(is, numNonWhiteRowEndChars);
	}
	skipNonblankChars(is, numNonWhiteEndChars);
}

void Formatter::fromStream(std::istream& is, ProjPoint& p) const
{
	skipNonblankChars(is, numNonWhiteStartChars);
	is >> p.x;
	skipNonblankChars(is, numNonWhiteElementSeparatorChars);
	is >> p.y;
	skipNonblankChars(is, numNonWhiteElementSeparatorChars);
	is >> p.z;
	skipNonblankChars(is, numNonWhiteElementSeparatorChars);
	is >> p.w;
	skipNonblankChars(is, numNonWhiteEndChars);
}

void Formatter::setElementSeparator(std::string sep)
{
	elementSeparator = sep;
	numNonWhiteElementSeparatorChars = countNumNonWhite(sep);
}

void Formatter::setRowSeparator(std::string sep)
{
	rowSeparator = sep;
	numNonWhiteRowSeparatorChars = countNumNonWhite(sep);
}

void Formatter::setRowStartEndStrings(std::string start, std::string end)
{
	rowStartString = start;
	numNonWhiteRowStartChars = countNumNonWhite(rowStartString);
	rowEndString = end;
	numNonWhiteRowEndChars = countNumNonWhite(rowEndString);
}

void Formatter::setStartEndStrings(std::string start, std::string end)
{
	startString = start;
	numNonWhiteStartChars = countNumNonWhite(startString);
	endString = end;
	numNonWhiteEndChars = countNumNonWhite(endString);
}

std::string Formatter::toString(double v) const
{
	std::ostringstream oss;
	oss << v;
	return oss.str();
}

std::string Formatter::toString(const AffPoint& p) const
{
	return startString + toString(p[X]) + elementSeparator +
		toString(p[Y]) + elementSeparator + toString(p[Z]) + endString; 
}

std::string Formatter::toString(const AffVector& v) const
{
	return startString + toString(v[DX]) + elementSeparator +
		toString(v[DY]) + elementSeparator + toString(v[DZ]) + endString; 
}

std::string Formatter::toString(const Matrix3x3& m) const
{
	std::string str = startString;
	for (int i=0 ; i<3 ; i++)
	{
		if (i > 0)
			str += rowSeparator;
		str += rowStartString;
		for (int j=0 ; j<3 ; j++)
		{
			if (j > 0)
				str += elementSeparator;
			str += toString(m.elementAt(i,j));
		}
		str += rowEndString;
	}
	str += endString;
	return str;
}

std::string Formatter::toString(const Matrix4x4& m) const
{
	std::string str = startString;
	for (int i=0 ; i<4 ; i++)
	{
		if (i > 0)
			str += rowSeparator;
		str += rowStartString;
		for (int j=0 ; j<4 ; j++)
		{
			if (j > 0)
				str += elementSeparator;
			str += toString(m.elementAt(i,j));
		}
		str += rowEndString;
	}
	str += endString;
	return str;
}

std::string Formatter::toString(const MatrixMxN& m) const
{
	int nr = m.getNumRows();
	int nc = m.getNumCols();
	std::string str = startString;
	for (int i=0 ; i<nr ; i++)
	{
		if (i > 0)
			str += rowSeparator;
		str += rowStartString;
		for (int j=0 ; j<nc ; j++)
		{
			if (j > 0)
				str += elementSeparator;
			str += toString(m.elementAt(i,j));
		}
		str += rowEndString;
	}
	str += endString;
	return str;
}

std::string Formatter::toString(const ProjPoint& p) const
{
	return startString +
		toString(p[X]) + elementSeparator +
		toString(p[Y]) + elementSeparator +
		toString(p[Z]) + elementSeparator +
		toString(p[W]) + endString; 
}

}
