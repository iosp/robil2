/*
 * RosTopicListener.h
 *
 *  Created on: Oct 27, 2013
 *      Author: blackpc
 */

#ifndef ROSTOPICLISTENER_H_
#define ROSTOPICLISTENER_H_

#include <iostream>
#include <set>
#include <python2.7/Python.h>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <scriptable_monitor/scripts/RosTopicListenerPython.h>
#include <scriptable_monitor/PythonExecuter.h>

#define foreach BOOST_FOREACH

#define debugPrint(str) std::cout << __FILE__  << ":" << __LINE__ << str << std::endl

//#define lock(mutexObject) 	debugPrint(" + Locking..."<<#mutexObject);\
//							boost::mutex::scoped_lock mutexObject##ScopedLock(mutexObject);\
//							debugPrint(" - Locked!"<<#mutexObject);

#define lock(mutexObject) boost::mutex::scoped_lock mutexObject##ScopedLock(mutexObject)
#define lock_recursive(mutexObject) boost::recursive_mutex::scoped_lock mutexObject##ScopedLock(mutexObject)

using namespace std;

class RosTopicListener
{
public:

	static void start();
	static void stop();
	static void addTopic(string topicName);
	static void removeTopic(string topicName);
	static vector<string> getTopics();
	static string getTopicValue(string topicName);
	static map<string, string> getTopicsValues();
	static bool hasTopicValue(string topicName);

private:
	static boost::mutex _activeTopicsMutex;
	static boost::thread* _listeningThread;
	static map<string, string> _topicValues;
	static set<string> _activeTopics;
	static PyMethodDef RosTopicPythonLib[];
	static bool			_running;

	static void listeningThread();

	static PyObject* internalInterruptionRequested(PyObject* self, PyObject* args);
	static PyObject* internalTopicUpdate(PyObject* self, PyObject* args);
	static PyObject* internalGetTopics(PyObject* self, PyObject* args);

};

#endif /* ROSTOPICLISTENER_H_ */
