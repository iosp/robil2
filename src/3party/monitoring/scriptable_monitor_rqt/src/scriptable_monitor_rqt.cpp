/*
 * scriptable_monitor_rqt.cpp
 *
 *  Created on: Oct 31, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor_rqt/scriptable_monitor_rqt.h>

namespace scriptable_monitor {

ScriptableMonitorRqt::ScriptableMonitorRqt()
	: rqt_gui_cpp::Plugin(), widget_(NULL), _nodeHandle(NULL), _newScriptDialog(NULL)
{

}

ScriptableMonitorRqt::~ScriptableMonitorRqt()
{
	delete _nodeHandle;
}

void ScriptableMonitorRqt::initPlugin(
		qt_gui_cpp::PluginContext& context)
{

	QStringList argv = context.argv();

	_nodeHandle = new ros::NodeHandle("~");
	_getScriptsClient = _nodeHandle->serviceClient<scriptable_monitor::GetScripts>("/scriptable_monitor/get_scripts", true);
	_addScriptClient = _nodeHandle->serviceClient<scriptable_monitor::AddScript>("/scriptable_monitor/add_script", true);

	_newScriptDialog = new QDialog(0, 0);
	widget_ = new QWidget();

	_scriptsUi.setupUi(widget_);
	_newScriptUi.setupUi(_newScriptDialog);

	context.addWidget(widget_);

	connectEvents();
	refreshScripts();
}

void ScriptableMonitorRqt::shutdownPlugin()
{

}

void ScriptableMonitorRqt::saveSettings(
		qt_gui_cpp::Settings& plugin_settings,
		qt_gui_cpp::Settings& instance_settings) const
{

}

void ScriptableMonitorRqt::connectEvents()
{
	// Main window
	connect(_scriptsUi.refreshButton, SIGNAL(clicked()), this, SLOT(refreshClicked()));
	connect(_scriptsUi.addScriptButton, SIGNAL(clicked()), this, SLOT(newScriptWindowClicked()));

	// Add script window
	connect(_newScriptUi.addButton, SIGNAL(clicked()), this, SLOT(addScriptClicked()));
}

void ScriptableMonitorRqt::restoreSettings(
		const qt_gui_cpp::Settings& plugin_settings,
		const qt_gui_cpp::Settings& instance_settings)
{

}

void ScriptableMonitorRqt::refreshScripts() {

	scriptable_monitor::GetScripts getScriptsService;
	_getScriptsClient.call(getScriptsService);
	vector<Script>& scripts = getScriptsService.response.scripts;

	if (_scriptsUi.scriptsTableView->model() != NULL)
		_scriptsUi.scriptsTableView->model()->deleteLater();

	QStandardItemModel* model = new QStandardItemModel(scripts.size(), 3);

	int row = 0;
	foreach(Script script, scripts) {

		model->setItem(row, 0, new QStandardItem(QString(script.name.c_str())));
		model->setItem(row, 1, new QStandardItem(QString("%1s").arg(script.interval)));
		model->setItem(row, 2, new QStandardItem(QString("")));
		row++;
	}

	model->setHeaderData(0, Qt::Horizontal, "Script name");
	model->setHeaderData(1, Qt::Horizontal, "Interval");
	model->setHeaderData(2, Qt::Horizontal, "Last execution");

	_scriptsUi.scriptsTableView->header()->setDefaultSectionSize(200);
	_scriptsUi.scriptsTableView->setModel(model);
}

}

PLUGINLIB_EXPORT_CLASS(scriptable_monitor::ScriptableMonitorRqt, rqt_gui_cpp::Plugin)
