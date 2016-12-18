/*
 * Printable.h
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */

#ifndef INCLUDE_COGNITAO_PRINTABLE_H_
#define INCLUDE_COGNITAO_PRINTABLE_H_

#include "core.h"
#include "Graph.h"

namespace cognitao{
namespace machine{

class Context;

class Printable{
public:
	virtual ~Printable(){};
	virtual
	void print(ostream& out, const Context& context)const=0;
	string tab(size_t n)const{
		stringstream s; while(s.str().size()<n) s<<" "; return s.str();
	}
	string tab(const Context& n)const;
};

}
}


#endif /* INCLUDE_COGNITAO_PRINTABLE_H_ */
