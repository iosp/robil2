Version 1.9 (2014 Jan 4)
	(*) More API cleanups and consistency changes for Matrix3x3 & Matrix4x4
	(*) Pulled tolerances out of Basic.h and placed them into a new
	    Tolerances.h to make certain alternate versions of utilities easier
	    to manage.

Version 1.8 (2013 Dec 30)
	(*) Lots of general cleanups, including removal of outdated entry points.
	(*) Elimination of now hopelessly out-of-date html documentation. (At some
	    point I hope to incorporate Doxygen (or some similar) comments into
	    header files.)
	(*) Removed IOSpec and its usages in AffPoint, AffVector, ProjPoint. As a
	    better replacement, added class Formatter that has similar functionality
	    and that can be used with those three types as well as the matrix types.
		(Idea is that this input/output formatting no longer clutters up the
	    geometry classes. Instead, Formatter does all its work outside the
	    implementation of those classes.)

Version 1.7 (2012 Jan 12)
	(*) Eliminated obsolete typedefs (Dxyz, Fxyz, et al.)
	(*) Added AffVector::vComponents methods.

Version 1.6 (2011 Aug 31)
	(*) Added some new entry points

Version 1.5 (2010 Aug 27)
	(*) Eliminated most uses of "friend"
	(*) Added AffPoint class methods to export arrays of float* or double*
		coordinates designed to feed OpenGL's glVertexPointer function. (See
		AffPoint::toCoordArray and AffPoint::toTransientCoordArray.)
	(*) Added AffVector class methods to export arrays of float* or double*
		coordinates designed to feed OpenGL's glNormalPointer function. (See
		AffVector::toCompArray and AffPoint::toTransientCompArray.)
	(*) Redefined definition of the AffPoint constructors that take arrays of
		coordinates. Formerly they assumed the array was length 4 and defined
		a point in projective space. This caused lots of confusion and errors.
		Now the array is assumed to be of length 3 with simple x, y, and z
		coordinates in affine space.

Version 1.4 (2010 Feb 10)
	Fixed bug in AffPoint::barycentricCoords

Version 1.3 (2010 Jan 11)
	Removed use of "real", but kept one typedef for backwards compatibility.
	Changed AffPoint and AffVector constructors to eliminate versions that,
		because of default parameters, allowed a single parameter constructor so
		that the following was syntactically (but not logically) valid:
			AffPoint p = (1,2,3)

Version 1.2 (2008 Oct 11):
	Fixed a bug in aPoint::distanceSquaredToLine
	Changed names of classes to be consistent with evolving Java version
		(*) includes more standard name capitalization conventions
		(*) Major changes: (aPoint -> AffPoint; aVector -> AffVector;
		    pPoint -> ProjPoint; viewVolume -> ViewVolume;
		    ioSpec -> IOSPec; packed_DArray -> Packed_DArray)
	Put all classes into cryph namespace

Version 1.1 (2007 Aug 30):
	Cleaned up std namespace usage. Eliminated support for compilers that
	do not support 'using' statements.

Version 1.0 (2006 Apr 28):
	Original Open Source version.

Version 0.0 (ca. 1996)
	Beginnings of original internal version.
