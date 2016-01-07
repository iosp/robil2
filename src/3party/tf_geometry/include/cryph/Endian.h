// Endian.h -- Class to test whether a particular host computer uses big or litle endian
//             data storage format. Also includes methods to swap bytes to switch "endian-ness".
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef ENDIAN_H
#define ENDIAN_H

/* Example of a typical usage:

 void correctForEndianDifferences(float* bufJustReadFromBinaryFile, int nFloatsRead,
                                  Endian::EndianType endianTypeOfMachineThatBuiltTheFile)
 {
	Endian::EndianType myHostEndianType = Endian::getEndian();
	if (myHostEndianType != endianTypeOfMachineThatBuiltTheFile)
		Endian::swapBytes(bufJustReadFromBinaryFile, nFloatsRead);
 }
 
 */

class Endian
{
public:
	union TwoByte
	{
		unsigned char b[2];
		short shortVal;
	};
	union FourByte
	{
		unsigned char b[4];
		float floatVal;
		int intVal;
	};
	union EightByte
	{
		unsigned char b[8];
		double doubleVal;
		long longVal;
	};
	
	enum EndianType { BIG, LITTLE };

	// Test what endian type this machine is...
	static EndianType getEndian()
	{
		Endian e;
		if (e.fourByte.floatVal != 1.0)
			return LITTLE;
		return BIG;
	}

	// generic byte swapping
	static void swapBytes(unsigned char* buf, int nBytes)
	{
		int other = nBytes-1;
		for (int i=0 ; i<nBytes/2 ; i++, other--)
		{
			unsigned char temp = buf[i];
			buf[i] = buf[other];
			buf[other] = temp;
		}
	}

	// byte swapping for two-byte objects...
	static void swapBytes(short& v)
	{
		Endian in, out;
		in.twoByte.shortVal = v;
		out.fourByte.b[0] = in.fourByte.b[1];
		out.fourByte.b[1] = in.fourByte.b[0];
		v = out.twoByte.shortVal;
	}

	// byte swapping for four-byte objects...
	static void swapBytes(float* v, int nV)
	{
		Endian in, out;
		for (int i=0 ; i<nV ; i++)
		{
			in.fourByte.floatVal = v[i];
			out.fourByte.b[0] = in.fourByte.b[3];
			out.fourByte.b[1] = in.fourByte.b[2];
			out.fourByte.b[2] = in.fourByte.b[1];
			out.fourByte.b[3] = in.fourByte.b[0];
			v[i] = out.fourByte.floatVal;
		}
	}
	static void swapBytes(float& v)
	{
		Endian in, out;
		in.fourByte.floatVal = v;
		out.fourByte.b[0] = in.fourByte.b[3];
		out.fourByte.b[1] = in.fourByte.b[2];
		out.fourByte.b[2] = in.fourByte.b[1];
		out.fourByte.b[3] = in.fourByte.b[0];
		v = out.fourByte.floatVal;
	}
	static void swapBytes(int& v)
	{
		Endian in, out;
		in.fourByte.intVal = v;
		out.fourByte.b[0] = in.fourByte.b[3];
		out.fourByte.b[1] = in.fourByte.b[2];
		out.fourByte.b[2] = in.fourByte.b[1];
		out.fourByte.b[3] = in.fourByte.b[0];
		v = out.fourByte.intVal;
	}

	// byte swapping for eight-byte objects...
	static void swapBytes(double* v, int nV)
	{
		Endian in, out;
		for (int i=0 ; i<nV ; i++)
		{
			in.eightByte.doubleVal = v[i];
			out.eightByte.b[0] = in.eightByte.b[7];
			out.eightByte.b[1] = in.eightByte.b[6];
			out.eightByte.b[2] = in.eightByte.b[5];
			out.eightByte.b[3] = in.eightByte.b[4];
			out.eightByte.b[4] = in.eightByte.b[3];
			out.eightByte.b[5] = in.eightByte.b[2];
			out.eightByte.b[6] = in.eightByte.b[1];
			out.eightByte.b[7] = in.eightByte.b[0];
			v[i] = out.eightByte.doubleVal;
		}
	}
	static void swapBytes(double& v)
	{
		Endian in, out;
		in.eightByte.doubleVal = v;
		out.eightByte.b[0] = in.eightByte.b[7];
		out.eightByte.b[1] = in.eightByte.b[6];
		out.eightByte.b[2] = in.eightByte.b[5];
		out.eightByte.b[3] = in.eightByte.b[4];
		out.eightByte.b[4] = in.eightByte.b[3];
		out.eightByte.b[5] = in.eightByte.b[2];
		out.eightByte.b[6] = in.eightByte.b[1];
		out.eightByte.b[7] = in.eightByte.b[0];
		v = out.eightByte.doubleVal;
	}
	static void swapBytes(long& v)
	{
		Endian in, out;
		in.eightByte.longVal = v;
		out.eightByte.b[0] = in.eightByte.b[7];
		out.eightByte.b[1] = in.eightByte.b[6];
		out.eightByte.b[2] = in.eightByte.b[5];
		out.eightByte.b[3] = in.eightByte.b[4];
		out.eightByte.b[4] = in.eightByte.b[3];
		out.eightByte.b[5] = in.eightByte.b[2];
		out.eightByte.b[6] = in.eightByte.b[1];
		out.eightByte.b[7] = in.eightByte.b[0];
		v = out.eightByte.longVal;
	}
	
	static void loadTwoByte(unsigned char* byteBuf, int& start, TwoByte& tb)
	{
		tb.b[0] = byteBuf[start];
		tb.b[1] = byteBuf[start+1];
	}
	static void loadFourByte(unsigned char* byteBuf, int& start, FourByte& fb)
	{
		for (int i=0 ; i<4 ; i++)
			fb.b[i] = byteBuf[start+i];
	}
	static void loadEightByte(unsigned char* byteBuf, int& start, EightByte& eb)
	{
		for (int i=0 ; i<8 ; i++)
			eb.b[i] = byteBuf[start+i];
	}

private:
	TwoByte twoByte;
	FourByte fourByte;
	EightByte eightByte;

	Endian()
	{
		// representation of 1.0 on big-endian machine (i.e.,
		// as on machine that generated the isabel data set.)
		fourByte.b[0] = 63; fourByte.b[1] = -128; fourByte.b[2] = fourByte.b[3] = 0;

		// we don't need to initialize eightByte to anything...
	}
};

#endif
