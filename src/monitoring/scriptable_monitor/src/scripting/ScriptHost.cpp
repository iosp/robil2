/*
 * ScriptHost.cpp
 *
 *  Created on: Oct 27, 2013
 *      Author: blackpc
 */

#include <scriptable_monitor/ScriptHost.h>
#include <scriptable_monitor/ScriptParameters.h>
#include "plp/PLP.h"
#include <scriptable_monitor/PlpModule.h>

ScriptHost::ScriptHost()
{
	_executionInterval = 0.1;
	PlpModule::sh = this;
	RosTopicListener::start();
}

ScriptHost::~ScriptHost()
{
	stop();
	RosTopicListener::stop();
}

bool ScriptHost::typeAnalization(string sourceCode, AddScriptResponse& response){
	ScriptParameters params(sourceCode);
	if(params["type"] == "predicate" or params["type"] == "" or params["type"] == "python"){
		// /*DEBUG]*/ cout<<"[i] predicate or python script is detected"<<endl;
		return true;
	}
	if(params["type"] == "plp"){
		// /*DEBUG]*/ cout<<"[i] plp script is detected"<<endl;
		stringstream file(sourceCode);
		PLPParser parser(file);
		parser.read_all();
		bool res = parser.create_structures();
		if(res){
			// /*DEBUG]*/ cout<<"PLP SCRIPT\n"<<parser.plp();cout<<endl;
			PLPCompiler compiler;
			int error_code=0;
			vector<MonitorningScript> ms = compiler.compile(parser.plp(), error_code);
			if(not error_code){
				string modulename="";
				for(size_t i=0;i<ms.size();i++){
					stringstream predicat_script; predicat_script<<ms[i];
					ScriptParameters p(predicat_script.str());
					modulename = p["module"];
					PlpModule::add_script(predicat_script.str());
				}
				// /*DEBUG]*/ parser.plp().repeat_frequency = "1.0";
				if(parser.plp().repeat_frequency.empty()==false){
					PlpModule::set_repeated_freq(modulename, boost::lexical_cast<double>(parser.plp().repeat_frequency));
				}
				PlpModule::start(modulename);

			}else{
				cerr << "[e] PLP script compilation problem, cannot add to ScriptHost" << endl;
				response.message = "[e] PLP script compilation problem, cannot add to ScriptHost";
				response.success = false;
			}
		}else{
			cerr << "[e] PLP script parsing problem, cannot add to ScriptHost" << endl;
			response.message = "[e] PLP script parsing problem, cannot add to ScriptHost";
			response.success = false;
		}
		return false;
	}
	cerr << "[e] Unknown script type, cannot add to ScriptHost" << endl;
	response.message = "[e] Unknown script type, cannot add to ScriptHost";
	response.success = false;
	return false;
}

AddScriptResponse ScriptHost::addScript(string sourceCode){
	AddScriptResponse response;
	response.message = "";
	return addScript(sourceCode, response);
}

AddScriptResponse ScriptHost::addScript(string sourceCode, AddScriptResponse& response)
{
	if(not typeAnalization(sourceCode,response)) return response;

	lock_recursive(_scriptsMutex);

	/*DEBUG]*/ if(false)
	{
		cout<<"======== add script ================="<<endl;
		cout<<sourceCode<<endl;
		cout<<"======== ========== ================="<<endl;
	}

	// /*DEBUG]*/ cout << "[+] Script added [ DEBUG ]" << endl;
	// /*DEBUG]*/ response.message = "[+] Script added";
	// /*DEBUG]*/ response.success = true;
	// /*DEBUG]*/ return response;

	ScriptParameters params(sourceCode);

	set<string> internalFunctions;
	foreach (string functionName, InternalFunctionsManager::getFunctionNames()) {
		internalFunctions.insert(functionName);
	}

	/**
	 * Create predicate script from the source code
	 */
	PredicatesScript predicateScript(sourceCode, internalFunctions);
	if (predicateScript.getName() == "") {
		cerr << "[e] Unnamed script provided, cannot add to ScriptHost" << endl;
		response.message = "[e] Unnamed script provided, cannot add to ScriptHost";
		response.success = false;
		return response;
	}

	if (scriptExists(predicateScript.getName())) {
		cerr << "[e] Script with the same name already exists" << endl;
		response.message = "[e] Script with the same name already exists";
		response.success = false;
		return response;
	}

	/**
	 * Convert to python script, simulate execution to extract used topic names
	 */
	PythonScript* pythonScript = new PythonScript(predicateScript.getPythonScript());
	// /*DEBUG]*/ cout<<"=== PYTHON SCRIPT ================================ \n"<<pythonScript->getSourceCode()<<"\n=============================="<<endl;
	bool validScript = prepareScript(*pythonScript);

	if (validScript)
		_scripts.insert(boost::shared_ptr<PythonScript>(pythonScript));
	else {
		cerr << "[e] Invalid script" << endl;
		response.message = "[e] Invalid script";
		response.success = false;
		return response;
	}

	/**
	 * Add used topics to listener
	 */
	foreach (string topicName, pythonScript->getUsedTopics()) {
		RosTopicListener::addTopic(topicName);
	}

	cout << "[+] Script added [" << pythonScript->getName() << "]" << endl;
	response.message = "[+] Script added";
	response.success = true;
	return response;
}

bool ScriptHost::prepareScript(PythonScript& script)
{
	CompilationResult result = _executer.compile(script);
	if (!result.success) {
		cerr << "[e] Compilation of script '" << script.getName() << "' failed" << endl;
		cerr << "[e] Message: " << result.message << endl;
		return false;
	}

	/**
	 * Extracts used topics, validations, internal function etc...
	 */
	_executer.simulate(script);

	return true;
}

void ScriptHost::run()
{
	while (!_workThread->interruption_requested()) {
		boost::system_time wait_time = get_system_time() + boost::posix_time::milliseconds(1000.0 * _executionInterval);

		{
			lock_recursive(_scriptsMutex);
			foreach (PythonScriptPtr script, _scripts) {

				if (!isExecutionTime(script))
					continue;

				if (!hasAllTopicValues(script))
					continue;

				// /*DEBUG]*/ cout << "[i] Time to execute '" << script->getName() << "'" << endl;
				_executer.execute(*script, RosTopicListener::getTopicsValues());
				script->updateExecutionTime();

				// /*DEBUG]*/ cout << "[i] Validations: " << (script->isValidationFailed() ? "FAILED [" + script->getFailedValidation() + "]" : "PASSED") << endl;

				addDiagnosticStatus(script);

			}
		}

		checkTimers();

		boost::this_thread::sleep(wait_time);
	}
}

void ScriptHost::start()
{
	RosTopicListener::start();

	if (!_workThread)
		_workThread = boost::shared_ptr<boost::thread>(
				new boost::thread(boost::bind(&ScriptHost::run, this)));
}

void ScriptHost::stop()
{
	ROS_INFO("Stopping ScriptHost...");
	_workThread->interrupt();
	_workThread->join();
}

bool ScriptHost::isExecutionTime(PythonScriptPtr script)
{
	boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime nextExecutionTime
		= script->getExecutionTime() + boost::posix_time::time_duration(boost::posix_time::milliseconds(1000.0 * script->getInterval()));

	return nowTime > nextExecutionTime;
}

bool ScriptHost::typeAnalizationForRemove(string name)
{
	lock_recursive(_scriptsMutex);
	if(PlpModule::contains(name)){
		PlpModule::stop(name);
		// /*DEBUG]*/ cout<<"[i] ... module stopped ("<<name<<")"<<endl;
		return true;
	}
	return false;
}
void ScriptHost::deleteScript(string scriptName)
{
	lock_recursive(_scriptsMutex);
	// /*DEBUG]*/ cout << "[i] deleteScript("<<scriptName<<")"<<endl;
	if(typeAnalizationForRemove(scriptName)){
		//cout<<"[i] deleted module is ("<<scriptName<<")"<<endl;
		return;
	}
	// /*DEBUG]*/ cout<<"[i] delete reguliar script ("<<scriptName<<")"<<endl;

	PythonScriptPtr script = getScript(scriptName);

	if (!scriptExists(scriptName)){
		cout<<"[e] Script "<<scriptName<<" cann't be deleted, it doesn't exists."<<endl;
		return;
	}

	_scripts.erase(script);
	cout << "[-] Script removed [" << scriptName << "]" << endl;

}

void ScriptHost::addDiagnosticStatus(PythonScriptPtr script)
{
	lock_recursive(_statusesMutex);

	DiagnosticStatusPtr status(new diagnostic_msgs::DiagnosticStatus());

	status->name = script->getName();
	status->hardware_id = script->getParameter("hardware_id");

	if (script->isValidationFailed()) {
		status->level = (int8_t)script->getFailType();
	}
	else
		status->level = diagnostic_msgs::DiagnosticStatus::OK;

	status->message = script->isValidationFailed() ? "Validation failed: " + script->getFailedValidation() : "Fine";;

	_diagnosticStatuses.push_back(status);
}

void ScriptHost::addDiagnosticStatus(string name, string hid, int8_t level, string message)
{
	cout<<"[i] DIAGNOSTIC("<<name<<","<<hid<<","<<(int)level<<","<<message<<")"<<endl;
	lock_recursive(_statusesMutex);

	DiagnosticStatusPtr status(new diagnostic_msgs::DiagnosticStatus());

	status->name = name;
	status->hardware_id = hid;

	status->level = level;

	status->message = message;

	_diagnosticStatuses.push_back(status);
}

/***
 * Check timers. When timer is timeout, the command associated with this timer executed.
 */
void ScriptHost::checkTimers(){
	static double time_from_start = system_time_seconds();
	//GET ALL TIMER READY FOR EXECUTION
	vector<PlpModule::Timer> timers = PlpModule::check_timers_for_timeout(true);
	// /*DEBUG]*/ cout<<"[i] system time = "<<system_time_seconds()-time_from_start<<" sec"<<endl;
	//EXECUTE COMMANDS OR ALL READY TIMERS
	BOOST_FOREACH(PlpModule::Timer t, timers){
		/*DEBUG]*/ cout<<"[i] .... timer = "<<t.name<<": "<<t.start-time_from_start<<" , timeout="<<t.timeout-time_from_start<<""<<endl;
		//REPORT COMMAND
		if(boost::starts_with(t.action,"report ")){
			int8_t level = -1; size_t plen=0;
			if(boost::starts_with(t.action,"report error ")){ level = diagnostic_msgs::DiagnosticStatus::ERROR; plen = string("report error ").length(); }
			if(boost::starts_with(t.action,"report warning ")){ level = diagnostic_msgs::DiagnosticStatus::WARN; plen = string("report warning ").length(); }
			if(boost::starts_with(t.action,"report info ")){ level = diagnostic_msgs::DiagnosticStatus::OK; plen = string("report info ").length(); }
			if(level>=0){
				addDiagnosticStatus(t.name,"",level,t.action.substr(plen));
			}
		}
	}
}

vector<DiagnosticStatusPtr> ScriptHost::getDiagnosticStatusesAndClear()
{
	lock_recursive(_statusesMutex);

	vector<DiagnosticStatusPtr> statuses = _diagnosticStatuses;
	_diagnosticStatuses.clear();
	return statuses;
}

set<PythonScriptPtr> ScriptHost::getScripts()
{
	lock_recursive(_scriptsMutex);
	return _scripts;
}

PythonScriptPtr ScriptHost::getScript(string scriptName)
{
	lock_recursive(_scriptsMutex);

	for(std::set<PythonScriptPtr>::iterator it = _scripts.begin(); it != _scripts.end(); it++) {
		if ((*it)->getName() == scriptName)
			return (*it);
	}

	return PythonScriptPtr();
}

bool ScriptHost::hasAllTopicValues(PythonScriptPtr script)
{
	foreach(string topicName, script->getUsedTopics()) {
		if (!RosTopicListener::hasTopicValue(topicName))
			return false;
	}

	return true;
}

bool ScriptHost::scriptExists(string scriptName)
{
	lock_recursive(_scriptsMutex);
	return !(!getScript(scriptName));
}


void ScriptHost::pauseModule(string scriptName)
{
	PlpModule::pause(scriptName);
}

void ScriptHost::resumeModule(string scriptName)
{
	PlpModule::resume(scriptName);

}







