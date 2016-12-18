/*
 * EventTranslator.cpp
 *
 *  Created on: Jun 29, 2014
 *      Author: dan
 */

#include "EventTranslator.h"

void EventTranslator(ComponentMain* comp, cognitao::bus::EventQueue* events)
{
#	define TRIGGER cognitao::bus::Event event=events->waitEvent(); if(event == cognitao::bus::Event()) return; if(false){}
#	define ON_EVENT(event_name) else if(event==cognitao::bus::Event(event_name))
#	define SEND(event_name) events->rise(cognitao::bus::Event(event_name))
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
			SEND("/smme/mission/./PauseMission");
		}


	}

#	undef TRIGGER
#	undef ON_EVENT
#	undef SEND
}

