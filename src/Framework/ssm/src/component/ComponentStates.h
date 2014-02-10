#ifndef COMPONENTSTATES_H_
#define COMPONENTSTATES_H_

#include "ComponentMain.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>

void init_dm(int argc, char** argv);
void startComponent(ComponentMain* component);
void startSystem(ComponentMain* component);

static void runComponent(int argc, char** argv, ComponentMain* component){
	init_dm(argc, argv);
	boost::thread_group th;
	th.add_thread( new boost::thread(boost::bind(&startSystem,component)) );
	th.add_thread( new boost::thread(boost::bind(&startComponent,component)) );
	th.join_all();
}

#endif /* COMPONENTSTATES_H_ */
