/*
 * EventTranslator.cpp
 *
 *  Created on: Jun 29, 2014
 *      Author: dan
 */

#include "EventTranslator.h"
#include <decision_making/EventSystem.h>
#include "ComponentMain.h"

void EventTranslator(ComponentMain* comp, decision_making::EventQueue* events)
{
	using namespace decision_making;
#	define TRIGGER Event event=events->waitEvent(); if(not event) return; if(false){}
#	define ON_EVENT(event_name) else if(event==Event(event_name))
#	define SEND(event_name) events->raiseEvent(Event(event_name))
#define X(x) x

	while(true){

		TRIGGER
		ON_EVENT("/Teleoperation"){
			SEND("/llc/Standby");
		}
		ON_EVENT("/Autonomy"){
			SEND("/llc/Resume");
		}
		ON_EVENT("/IEDDetected"){
			SEND("/PauseMission");
		}


	}

#	undef TRIGGER
#	undef ON_EVENT
#	undef SEND
}

