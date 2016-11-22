/*
 * EventProcessor.h
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */

#ifndef INCLUDE_COGNITAO_EVENTPROCESSOR_H_
#define INCLUDE_COGNITAO_EVENTPROCESSOR_H_

#include "core.h"
#include "Event.h"

namespace cognitao{
namespace machine{

class Events;

class EventProcessor{
public:
	virtual
	~EventProcessor(){}

	virtual
	void on_match(const Event& original, const Event& stranger)=0;

	virtual
	void send(const Event& original)=0;

	virtual
	void on_private(const Event& original)=0;


	void send(const Events& original);

	void on_private(const Events& original);

};

}
}


#endif /* INCLUDE_COGNITAO_EVENTPROCESSOR_H_ */
