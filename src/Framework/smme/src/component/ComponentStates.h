#ifndef COMPONENTSTATES_H_
#define COMPONENTSTATES_H_
#include "ComponentMain.h"
void runComponent(int argc, char** argv, ComponentMain& component);
void runComponent(int argc, char** argv, ComponentMain* component){runComponent(argc, argv, *component);}
#endif /* COMPONENTSTATES_H_ */
