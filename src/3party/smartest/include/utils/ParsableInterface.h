/*
 * ParsableInterface.h
 *
 *  Created on: Mar 26, 2014
 *      Author: userws1
 */

#ifndef PARSABLEINTERFACE_H_
#define PARSABLEINTERFACE_H_
#include "TinyXmlDef.h"

class ParsableInterface{

public:
	inline virtual ~ParsableInterface(){}
	virtual TiXmlElement *toXMLElement()=0;
	virtual void fromXMLElement(TiXmlElement * node)=0;
};

#endif /* PARSABLEINTERFACE_H_ */
