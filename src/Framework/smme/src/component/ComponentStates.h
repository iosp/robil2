#ifndef COMPONENTSTATES_H_
#define COMPONENTSTATES_H_
#include "ComponentMain.h"

void runComponent(int argc, char** argv, ComponentMain& component);

inline
void runComponent(int argc, char** argv, ComponentMain* component){runComponent(argc, argv, *component);}

void startSystem(ComponentMain* component);

//==============================================================================================

void initMissionTasks();
void initTask();
void startStateService(ComponentMain* component, std::string mid);


class TaskMachine{
	ComponentMain* component;
	std::string mission_id;
	MissionManager* manager;
	boost::thread_group thread;
	void* events_ptr;
public:
	TaskMachine():component(0),manager(0),events_ptr(0){}
	TaskMachine(ComponentMain* comp,std::string mid):component(comp),mission_id(mid),events_ptr(0){
		manager = component->mission_manager();
	}
	void start(){
		thread.add_thread( new boost::thread(boost::bind(&TaskMachine::startTask, this, component, mission_id)) );
	}
	void stop();
	void startTask(ComponentMain* component, std::string mission_id);
};

class MissionMachine{
	boost::thread_group thread;
	ComponentMain* component;
	std::string mission_id;
	TaskMachine task;
	MissionManager* manager;
	void* events_ptr;

	static int init(ComponentMain* comp,std::string mid){
		startStateService(comp,mid);
		initMissionTasks();
		initTask();
		return 0;
	}
public:
	MissionMachine():component(0),manager(0),events_ptr(0){}
	MissionMachine(ComponentMain* comp,std::string mid);
	void start(){
		thread.add_thread( new boost::thread(boost::bind(&MissionMachine::startMission,this,component, mission_id)) );
		task.start();
	}
	void stop();
	void startMission(ComponentMain* component, std::string mission_id);
};


















#endif /* COMPONENTSTATES_H_ */
