#ifndef COMPONENTSTATES_H_
#define COMPONENTSTATES_H_
#include "ComponentMain.h"

void runComponent(int argc, char** argv, ComponentMain& component);

inline
void runComponent(int argc, char** argv, ComponentMain* component){runComponent(argc, argv, *component);}

void startSystem(ComponentMain* component);

void initMissionTasks();
void startMission(ComponentMain* component, std::string mission_id);
void startTask(ComponentMain* component);

inline
boost::thread* startMissionThread(ComponentMain* component, std::string mission_id){
	return boost::thread(boost::bind(startMission, component, mission_id));
}
inline
boost::thread* startTaskThread(ComponentMain* component){
	return boost::thread(boost::bind(startTask, component));
}

#endif /* COMPONENTSTATES_H_ */
