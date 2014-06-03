/*
 * main.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: blackpc
 */


#include <python2.7/Python.h>
#include <scriptable_monitor/scriptable_monitor.h>
#include <iostream>
#include <fstream>

#include <scriptable_monitor/ScriptHost.h>
#include <scriptable_monitor/SomeInternalFunctionExample.h>


using namespace std;

void rosTopicListenerTest() {
	cout << "Starting..." << endl;

	RosTopicListener::start();
	sleep(3);

	RosTopicListener::addTopic("/scan/ranges[0:1]");
	RosTopicListener::addTopic("/scan/ranges[1:2]");
	RosTopicListener::addTopic("/scan/ranges[2:3]");


	sleep(7);
	RosTopicListener::stop();

	cout << "Values:" << endl;
	cout << RosTopicListener::getTopicValue("/scan/ranges[0:1]") << endl;
	cout << RosTopicListener::getTopicValue("/scan/ranges[1:2]") << endl;
	cout << RosTopicListener::getTopicValue("/scan/ranges[2:3]") << endl;

	cout << "Done" << endl;
}

void internalFunctionsTest() {
	vector<string> names = InternalFunctionsManager::getFunctionNames();

	cout << "Available functions:" << endl;

	for (size_t i = 0; i < names.size(); i++)
		cout << " - " << names[i] << endl;
}

void scriptingTest() {
	string s =
			"#! name script_name\n"
			"#! type predicate\n"
			"#! author Bla bla bablalovich\n"
			"core1 = {/diagntostic/cpu/core1}\n"
			"core2 = {/diagntostic/cpu/core2}\n"
			"core3 = {/diagntostic/cpu/core3}\n"
			"core4 = {/diagntostic/cpu/core4}\n"
			"cpu_th = {/config/cpu/cpu_threshold}\n"
			"core_th = {/config/cpu/core_threshold}\n"
			"average = average(core1, core2, core3, core4)\n"
			"average < cpu_th\n"
			"core1 < core_th\n"
			"core2 < core_th\n"
			"core3 < core_th\n"
			"core4 < core_th\n";

	string errorScript =
			"#! name script_name2\n"
			"#! type predicate\n"
			"core1x x core2x\n";

	set<string> internalFunctions;
	internalFunctions.insert("average");

	map<string, string> topics;
	topics["/diagntostic/cpu/core1"] = "1";
	topics["/diagntostic/cpu/core2"] = "1";
	topics["/diagntostic/cpu/core3"] = "1";
	topics["/diagntostic/cpu/core4"] = "1";
	topics["/diagntostic/cpu/cpu"] = "1";
	topics["/config/cpu/cpu_threshold"] = "20";
	topics["/config/cpu/core_threshold"] = "20";

	PredicatesScript ps(s, internalFunctions);
	PredicatesScript psError(errorScript, internalFunctions);

	PythonScript py = ps.getPythonScript();
	PythonScript pyError = psError.getPythonScript();

	ScriptExecuter pe;
	CompilationResult result;

	result = pe.compile(pyError);
	cout << "Compilation result = " << (result.success ? "SUCCESS" : "FAILED") << endl;

	result = pe.compile(py);
	cout << "Compilation result = " << (result.success ? "SUCCESS" : "FAILED") << endl;

//	pe.simulate(py);
//	pe.execute(py, topics);
}

namespace{

	string read_file(string file_name){
		ifstream file(file_name.c_str());
		if(file){
			char c; stringstream s;
			while(file.eof()==false){
				file.read(&c,1);
				s<<c;
			}
			return s.str();
		}
		return "";
	}

}

void scriptHostTest() {

	ScriptHost host;

	string s1 =	"#! type predicate\n"
				"#! name predicate_script1\n"
				"#! interval 2\n"
				"core1 = {/scan/ranges[0:1]}\n"
				"core2 = {/scan/ranges[1:2]}\n"
				"core3 = {/scan/ranges[2:3]}\n"
				"core4 = {/scan/ranges[3:4]}\n"
				"cpu_th = {/scan/ranges[4:5]}\n"
				"core_th = {/scan/ranges[5:6]}\n"
				"average = SomeFunction(core1, core2, core3, core4)\n"
				"average < cpu_th\n"
				"core1 < core_th\n"
				"core2 < core_th\n"
				"core3 > core_th\n"
				"core4 < core_th\n";

	string s2 =	"#! type predicate\n"
				"#! name predicate_script2\n"
				"#! interval 3\n"
				"core1 = {/scan/ranges[0:1]}\n"
				"core2 = {/scan/ranges[1:2]}\n"
				"core3 = {/scan/ranges[2:3]}\n"
				"core4 = {/scan/ranges[3:4]}\n"
				"cpu_th = {/scan/ranges[4:5]}\n"
				"core_th = {/scan/ranges[5:6]}\n"
				"average = SomeFunction(core1, core2, core3, core4)\n"
				"average < cpu_th\n"
				"core1 < core_th\n"
				"core2 < core_th\n"
				"core3 > core_th\n"
				"core4 < core_th\n";

	string plp_header = "#! type plp\n"
						"#! name plp_test4\n"
						"#! interval 3\n";
	string plp = plp_header +
				"PLP: module1\n"
				"Type: Achieve module\n"
				"";

	string plp1 =
			"#! type predicate\n"
			"#! name Module_1_Resource_precondition\n"
			"#! module Module_1\n"
			"#! time on_start\n"
			"Power_source = {/scan/ranges[3]}\n"
			"_tmp = set_global_var('InitSourceData_Module_1_Power',Power_source)\n"
			"20 <= Power_source\n"
			"print 'on time script ', 'Module_1_Resource_precondition'\n"
			"_tmp = remove_script('Module_1','Module_1_Resource_precondition')\nprint 'End'\n";

	string plp_test4 = plp_header + read_file("test4.plp");
	//cout<<"TEST4"<<endl<<plp_test4<<endl;

	host.start();
	cout << "Running scripts" << endl;

	sleep(1);

//	host.addScript(s1);
//	host.addScript(s2);
//	host.addScript(plp);
	host.addScript(plp_test4);
//	host.addScript(plp1);

	sleep(1);
	host.pauseModule("Module_1");
	sleep(1);
	host.resumeModule("Module_1");
	sleep(1);

	cout<<"-------------------------------------------------"<<endl;
	host.deleteScript("Module_1");

	sleep(5);
	cout<<"Finish..."<<endl;

	host.stop();

	cout << "Done" << endl;
}

void yamlTest() {
	 stringstream yaml_stream;
	 yaml_stream << "[1.5462267]";
	 YAML::Parser parser(yaml_stream);
	 YAML::Node document;
	 parser.GetNextDocument(document);

	 cout << document.size() << endl;
}

int main(int argc, char **argv) {
//	RosTopicListener::start();

//	yamlTest();
//	return 0;

	scriptHostTest();
	return 0;

//	internalFunctionsTest();
//	return 0;
//
//	rosTopicListenerTest();
//	return 0;

//	scriptingTest();
//	return 0;

}
