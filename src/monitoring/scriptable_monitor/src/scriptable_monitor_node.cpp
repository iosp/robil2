/*
 * Filename: scriptable_monitor_node.cpp
 *   Author: Igor Makhtes
 *     Date: Dec 4, 2013
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Cogniteam Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <scriptable_monitor/ScriptableMonitorNode.h>

using namespace std;

#define foreach BOOST_FOREACH

void diagnosticPublisher(ros::NodeHandle& node, ScriptHost& scriptHost) {

	ros::Publisher publisher = node.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100, false);

	while (ros::ok()) {
		diagnostic_msgs::DiagnosticArray array;
		array.header.stamp = ros::Time::now();

		foreach(DiagnosticStatusPtr status, scriptHost.getDiagnosticStatusesAndClear()) {
			array.status.push_back(*status);
		}

		publisher.publish(array);

		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "scriptable_monitor");
	ros::NodeHandle node("~");
	ScriptableMonitorNode scriptNode;
	boost::thread diagnosticPublishThread(boost::bind(diagnosticPublisher, boost::ref(node), boost::ref(scriptNode.getScriptHost())));
	ros::spin();
	return 0;
}
