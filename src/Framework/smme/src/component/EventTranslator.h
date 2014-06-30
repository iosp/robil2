/*
 * EventTranslator.h
 *
 *  Created on: Jun 29, 2014
 *      Author: dan
 */

#ifndef EVENTTRANSLATOR_H_
#define EVENTTRANSLATOR_H_

class ComponentMain;
namespace decision_making{ class EventQueue; }

void EventTranslator(ComponentMain* comp, decision_making::EventQueue* events);

#endif /* EVENTTRANSLATOR_H_ */
