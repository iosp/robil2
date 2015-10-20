#include "boost/assign.hpp"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/odeint.hpp>
template<typename T>
std::vector<T> polyval( const std::vector<T>& oCoeff, 
	const std::vector<T>& oX )
{
	size_t nCount =  oX.size();
	size_t nDegree = oCoeff.size();
	std::vector<T>	oY( nCount );
 
	for ( size_t i = 0; i < nCount; i++ )
	{
		T nY = 0;
		T nXT = 1;
		T nX = oX[i];
		for ( size_t j = 0; j < nDegree; j++ )
		{
			// multiply current x by a coefficient
			nY += oCoeff[j] * nXT;
			// power up the X
			nXT *= nX;
		}
		oY[i] = nY;
	}
 
	return oY;
}

template<typename T>
std::vector<T> polyfit( const std::vector<T>& oX, 
	const std::vector<T>& oY, int nDegree )
{
	using namespace boost::numeric::ublas;
 
	if ( oX.size() != oY.size() )
		throw std::invalid_argument( "X and Y vector sizes do not match" );
 
	// more intuative this way
	nDegree++;
	
	size_t nCount =  oX.size();
	matrix<T> oXMatrix( nCount, nDegree );
	matrix<T> oYMatrix( nCount, 1 );
	
	// copy y matrix
	for ( size_t i = 0; i < nCount; i++ )
	{
		oYMatrix(i, 0) = oY[i];
	}
 
	// create the X matrix
	for ( size_t nRow = 0; nRow < nCount; nRow++ )
	{
		T nVal = 1.0f;
		for ( int nCol = 0; nCol < nDegree; nCol++ )
		{
			oXMatrix(nRow, nCol) = nVal;
			nVal *= oX[nRow];
		}
	}
 
	// transpose X matrix
	matrix<T> oXtMatrix( trans(oXMatrix) );
	// multiply transposed X matrix with X matrix
	matrix<T> oXtXMatrix( prec_prod(oXtMatrix, oXMatrix) );
	// multiply transposed X matrix with Y matrix
	matrix<T> oXtYMatrix( prec_prod(oXtMatrix, oYMatrix) );
 
	// lu decomposition
	permutation_matrix<int> pert(oXtXMatrix.size1());
	const std::size_t singular = lu_factorize(oXtXMatrix, pert);
	// must be singular
	BOOST_ASSERT( singular == 0 );
 
	// backsubstitution
	lu_substitute(oXtXMatrix, pert, oXtYMatrix);
 
	// copy the result to coeff
	return std::vector<T>( oXtYMatrix.data().begin(), oXtYMatrix.data().end() );
}


