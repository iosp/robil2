/*
 * RosTopicListener.cpp
 *
 *  Created on: Oct 27, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/RosTopicListener.h>

bool RosTopicListener::_running = false;
boost::mutex RosTopicListener::_activeTopicsMutex;
boost::thread* RosTopicListener::_listeningThread = NULL;
map<string, string> RosTopicListener::_topicValues;
set<string> RosTopicListener::_activeTopics;
PyMethodDef RosTopicListener::RosTopicPythonLib[] =
{
	{"internal_interruption_requested", &RosTopicListener::internalInterruptionRequested, 		METH_NOARGS, ""	},
	{"internal_topic_update", 			&RosTopicListener::internalTopicUpdate, 				METH_VARARGS, ""},
	{"internal_get_topics", 			&RosTopicListener::internalGetTopics, 					METH_NOARGS, ""	},
	{NULL, 					NULL, 									0, 		NULL}
};

PyObject* RosTopicListener::internalTopicUpdate(PyObject* self, PyObject* args)
{
	string topicName  = PyString_AsString(PyTuple_GetItem(args, 0));
	string topicValue = PyString_AsString(PyTuple_GetItem(args, 1));

	_topicValues[topicName] = topicValue;

	return PyString_FromString("Ok");
}

PyObject* RosTopicListener::internalInterruptionRequested(PyObject* self, PyObject* args)
{
	if (_listeningThread == NULL)
		return PyBool_FromLong(0);

	return PyBool_FromLong((long)_listeningThread->interruption_requested());
}

PyObject* RosTopicListener::internalGetTopics(PyObject* self, PyObject* args)
{
	lock(_activeTopicsMutex);

	PyObject* topicsList = PyList_New(_activeTopics.size());

	int i = 0;
	foreach(string topic, _activeTopics) {
		PyList_SetItem(topicsList, i++, PyString_FromString(topic.c_str()));
	}

	return topicsList;
}

void RosTopicListener::start()
{
	if (_running)
		return;

	PythonExecuter::initialize();
	PythonExecuter::initModule("cogni_topic_listener", RosTopicPythonLib);

	_listeningThread = new boost::thread(listeningThread);
	sleep(1);

	_running = true;
}

void RosTopicListener::stop() {

	if (!_running)
		return;

	cout << "Interrupting..." << endl;
	_listeningThread->interrupt();
	cout << "Joining..." << endl;
	_listeningThread->join();
	cout << "Joined" << endl;
	_running = false;
}

void RosTopicListener::listeningThread()
{
	PythonExecuter::execute(ROS_TOPIC_LISTENER_SCRIPT);
}


void RosTopicListener::addTopic(string topicName)
{
	lock(_activeTopicsMutex);

	if (_activeTopics.count(topicName) > 0)
		return;

	_activeTopics.insert(topicName);
}

void RosTopicListener::removeTopic(string topicName)
{
	lock(_activeTopicsMutex);

	if (_activeTopics.count(topicName) > 0)
		return;

	_activeTopics.erase(topicName);
}

vector<string> RosTopicListener::getTopics()
{
	lock(_activeTopicsMutex);

	vector<string> topics;

	foreach(string t, _activeTopics) {
		topics.push_back(t);
	}

	return topics;
}

string RosTopicListener::getTopicValue(string topicName)
{
	if (_topicValues.count(topicName) == 0)
		return "";

	return _topicValues[topicName];
}

map<string, string> RosTopicListener::getTopicsValues()
{
	return _topicValues;
}

bool RosTopicListener::hasTopicValue(string topicName)
{
	return _topicValues.count(topicName) > 0;
}
