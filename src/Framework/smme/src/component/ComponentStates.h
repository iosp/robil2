#ifndef COMPONENTSTATES_H_
#define COMPONENTSTATES_H_
#include "ComponentMain.h"

void runComponent(int argc, char** argv, ComponentMain& component);

inline
void runComponent(int argc, char** argv, ComponentMain* component){runComponent(argc, argv, *component);}

void startSystem(ComponentMain* component);

void startMission(ComponentMain* component);

#endif /* COMPONENTSTATES_H_ */
