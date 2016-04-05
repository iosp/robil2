/*
 * core.h
 *
 *  Created on: Nov 22, 2015
 *      Author: dan
 */

#ifndef SRC_COGNITAO_COMPILER_CORE_H_
#define SRC_COGNITAO_COMPILER_CORE_H_

#include "../parser/core.h"
#include <cognitao/machine/Machine.h>

namespace cognitao {
namespace io {
namespace compiler {

typedef cognitao::io::parser::core::Node Node;

class CompilerError{
public:
	std::string message;
	CompilerError( std::string m = "" ):message(m){}
	template<class T>
	CompilerError operator<<( const T& t ) const
	{
		CompilerError p;
		stringstream s; s<<message<<" "<<t; p.message = s.str(); return p;
	}
};



} /* namespace compiler */
} /* io */
} /* namespace cognitao */

#endif /* SRC_COGNITAO_COMPILER_CORE_H_ */
