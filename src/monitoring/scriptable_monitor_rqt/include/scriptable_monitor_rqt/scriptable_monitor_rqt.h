/*
 * scriptable_monitor_rqt.h
 *
 *  Created on: Oct 31, 2013
 *      Author: blackpc
 */

#ifndef scriptable_monitor_RQT_H_
#define scriptable_monitor_RQT_H_

#include <iostream>

#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <QWidget>
#include <QStandardItemModel>
#include <QMessageBox>

#include <rqt_gui_cpp/plugin.h>
#include <pluginlib/class_list_macros.h>

#include <scriptable_monitor_rqt/ScriptsList.h>
#include <scriptable_monitor_rqt/AddScript.h>

#include <scriptable_monitor/Script.h>
#include <scriptable_monitor/GetScripts.h>
#include <scriptable_monitor/AddScript.h>

using namespace std;

namespace scriptable_monitor
{

class ScriptableMonitorRqt : public rqt_gui_cpp::Plugin
{

Q_OBJECT

public:
	ScriptableMonitorRqt();
	~ScriptableMonitorRqt();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
			qt_gui_cpp::Settings& instance_settings) const;
	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
			const qt_gui_cpp::Settings& instance_settings);

public slots:
	void refreshClicked() {
		refreshScripts();
	}

	void newScriptWindowClicked() {
		_newScriptDialog->show();

	}

	void addScriptClicked() {
		scriptable_monitor::AddScript addScriptService;

		addScriptService.request.script.data = _newScriptUi.scriptText->toPlainText().toStdString();
		_addScriptClient.call(addScriptService);

		if ((int)addScriptService.response.success == 1) {
			_newScriptDialog->hide();
			refreshScripts();
			return;
		}

		QMessageBox Msgbox;
		Msgbox.setText(QString::fromStdString("Couldn't add script: " + addScriptService.response.message.data));
		Msgbox.exec();
	}

private:
	QWidget* widget_;
	Ui_ScriptsForm _scriptsUi;
	Ui_NewScriptForm _newScriptUi;
	QDialog* _newScriptDialog;

	ros::NodeHandle* _nodeHandle;
	ros::ServiceClient _getScriptsClient;
	ros::ServiceClient _addScriptClient;

	void refreshScripts();
	void connectEvents();

};
} // namespace

#endif /* scriptable_monitor_RQT_H_ */
