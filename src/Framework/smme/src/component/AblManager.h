/*
 * AblManager.h
 *
 *  Created on: Mar 20, 2014
 *      Author: dan
 */

#ifndef ABLMANAGER_H_
#define ABLMANAGER_H_

#include "ComponentMain.h"

namespace decision_making{
class EventQueue;
}

class AblManager {
	ComponentMain* component;
	struct Trigger{
		std::string name;
		std::string compliment;
		Trigger(){}
		Trigger(std::string n, std::string c):name(n),compliment(c){}
		bool operator<(const Trigger& t)const{ return name<t.name; }
	};
	std::map<std::string,Trigger> all_triggers;
	//--- pre-processed structures
	std::map<std::string,std::string> all_compliments;
	std::set<std::string> all_triggers_names;

	struct Activated{
		Trigger trigger;
		std::string polyci;
	};
	std::list<Activated> activated;

public:
	AblManager(ComponentMain* comp);
	virtual ~AblManager();
	void listen(decision_making::EventQueue* events);

protected:
	bool is_abl_event(std::string event_name)const;
	bool is_trigger(std::string event_name)const;
	bool on_event(std::string event_name);
	bool on_trigger(std::string event_name);
	bool on_compliment(std::string event_name);
	void activate(const Trigger& trigger,const std::string& polyci);
	void deactivate(const std::string& compliment);
	void on_activation(const Activated& act);
	void on_deactivation(const Activated& act);
};

#endif /* ABLMANAGER_H_ */
