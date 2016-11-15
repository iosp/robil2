#include <iostream>
#include <ros/ros.h>
//#include <decision_making/SynchCout.h>

// OLD
//#include <decision_making/BT.h>
//#include <decision_making/FSM.h>
//#include <decision_making/ROSTask.h>
//#include <decision_making/DecisionMaking.h>
//#include <decision_making/DebugModeTracker.hpp>

#include <robil_msgs/Map.h>
#include <robil_msgs/MapCell.h>

using namespace std;
//using namespace decision_making;

#include "ComponentStates.h"
#include "WsmTask.h"

#define DELETE(X) if(X){delete X; X=NULL;}
#define RESET(X,Y) if(current_task == X) { \
					ROS_WARN_STREAM(" Current WSM task: " << current_task); \
					DELETE(task_ptr) \
					task_ptr = new Y; \
					task_ptr->start(); \
					continue;}
#define EVENT(X) \
		cognitao::bus::Event( \
				cognitao::bus::Event::name_t(X), \
				cognitao::bus::Event::channel_t(""), \
				cognitao::bus::Event::context_t(context))
#define RAISE(X) processor_ptr->bus_events << EVENT(X)

ComponentMain *Global_comp;

struct PauseStruct {
private:
	bool paused;
	boost::mutex io_mutex;
public:
	PauseStruct() :
			paused(false) {
	}

	void set_pause(bool new_pause) {
		boost::mutex::scoped_lock stdout_lock(io_mutex);
		paused = new_pause;
		if (paused) {
			cout << "\033[1;31m PAUSED \033[0m\n";
		} else
			cout << "\033[1;31m RESUMED \033[0m\n";
	}
	bool get_pause() {
		boost::mutex::scoped_lock stdout_lock(io_mutex);
		return paused;
	}
};
PauseStruct pause_time;

bool SensorConnection;
sensor_msgs::JointState jointStates;

//class Params: public CallContextParameters{
//public:
//	ComponentMain* comp;
//	Params(ComponentMain* comp):comp(comp){}
//	std::string str()const{return "";}
//};

void JointStatesCallback(const sensor_msgs::JointStateConstPtr &msg) {
	jointStates = sensor_msgs::JointState(*msg);
	Global_comp->jointStates = &jointStates;
}

//void pauseCallback(const std_msgs::StringConstPtr &msg)
//{
//		if((msg->data.find("ResumeTask",0) != -1)&&(pause_time)){
//			pause_time = false ;
//			if(Global_comp->cur_mission == NULL){
//			//	ROS_ERROR("Not task to resume/pause");
//				return;
//			}
//			if((Global_comp->cur_mission->Get_status() == "paused")){
//				Global_comp->cur_mission->Set_task_status("active");
//				return;
//			}
//			else{
//				ROS_ERROR("No Task to resume, has Task %d at status %s",Global_comp->cur_mission->Get_Task_id(),Global_comp->cur_mission->Get_status().c_str());
//				return;
//			}
//		}
//		if(Global_comp->cur_mission == NULL){
//					//ROS_ERROR("Not task to resume/pause");
//					return;
//				}
//		if((msg->data.find("PauseMission",0) != -1)&&(Global_comp->cur_mission->Get_status()=="active")){
//			pause_time = true ;
//			Global_comp->cur_mission->Set_task_status("paused");
//			return;
//		}
//		else{
//			//ROS_ERROR("No Task to pause, has Task %d at status %s",Global_comp->cur_mission->Get_Task_id(),Global_comp->cur_mission->Get_status().c_str());
//			return;
//		}
//		return;
//}

void pause_checker(cognitao::bus::Event msg, ComponentMain * comp_ptr) {
	if (comp_ptr == NULL)
		return;
	if ((msg.name() == "ResumeTask") && (pause_time.get_pause())) {
		pause_time.set_pause(false);
		if (comp_ptr->cur_mission == NULL) {
//			ROS_ERROR("Not task to resume/pause");
			return;
		}
		if ((comp_ptr->cur_mission->Get_status() == "paused")) {
			comp_ptr->cur_mission->Set_task_status("active");
		} else {
			ROS_ERROR("No Task to resume, has Task %d at status %s",
					comp_ptr->cur_mission->Get_Task_id(),
					comp_ptr->cur_mission->Get_status().c_str());
		}
		return;
	}
	if (comp_ptr->cur_mission == NULL) {
//		ROS_ERROR("Not task to resume/pause");
		return;
	}
	if ((msg.name() == "PauseMission")
			&& (comp_ptr->cur_mission->Get_status() == "active")) {
		pause_time.set_pause(true);
		comp_ptr->cur_mission->Set_task_status("paused");
	}
	return;
}

//FSM(wsm_WORK)
//{
//	FSM_STATES
//	{
//		STANDBY,
//		READY
//	}
//	FSM_START(STANDBY);
//	FSM_BGN
//	{
//		FSM_STATE(STANDBY)
//		{
//			FSM_CALL_TASK(STANDBY);
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/wsm/Resume", FSM_NEXT(READY));
//			}
//		}
//		FSM_STATE(READY)
//		{
//			FSM_CALL_TASK(READY);
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/wsm/Standby", FSM_NEXT(STANDBY));
//			}
//		}
//
//	}
//	FSM_END
//}
//
//FSM(wsm_ON)
//{
//	FSM_STATES
//	{
//		INIT,
//		WORK
//	}
//	FSM_START(INIT);
//	FSM_BGN
//	{
//		FSM_STATE(INIT)
//		{
//			FSM_CALL_TASK(INIT);
//			FSM_TRANSITIONS
//			{
//				//FSM_ON_CONDITION(SensorConnection, FSM_NEXT(WORK));
//				FSM_ON_EVENT("/wsm/SensorConnected", FSM_NEXT(WORK));
//			}
//		}
//		FSM_STATE(WORK)
//		{
//			FSM_CALL_FSM(wsm_WORK)
//			FSM_TRANSITIONS
//			{
//				//FSM_ON_CONDITION(not SensorConnection, FSM_NEXT(INIT));
//				FSM_ON_EVENT("/wsm/SensorNotConnected", FSM_NEXT(INIT));
//			}
//		}
//
//	}
//	FSM_END
//}
//
//FSM(wsm)
//{
//	FSM_STATES
//	{
//		OFF,
//		ON
//	}
//	FSM_START(ON);
//	FSM_BGN
//	{
//		FSM_STATE(OFF)
//		{
//			FSM_CALL_TASK(OFF);
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/Activation", FSM_NEXT(ON));
//				FSM_ON_EVENT("/wsm/Activation", FSM_NEXT(ON));
//			}
//		}
//		FSM_STATE(ON)
//		{
//			FSM_CALL_FSM(wsm_ON)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("/Shutdown", FSM_NEXT(OFF));
//				FSM_ON_EVENT("/wsm/Shutdown", FSM_NEXT(OFF));
//			}
//		}
//
//	}
//	FSM_END
//}

//TaskResult state_OFF(string id, const CallContext& context, EventQueue& events){
//
//	//diagnostic_msgs::DiagnosticStatus status;
//	//COMPONENT->publishDiagnostic(status);
//	ROS_INFO("WSM OFF");
//	return TaskResult::SUCCESS();
//}

//TaskResult state_INIT(string id, const CallContext& context, EventQueue& events){
//	//PAUSE(10000);
//	ROS_INFO("WSM INIT");
//
//	while(COMPONENT->receivedLocation == NULL){}
//	COMPONENT->z_offset = COMPONENT->receivedLocation->pose.pose.position.z ;
//	if(COMPONENT->z_offset < 0){
//		COMPONENT->z_offset = fabs(COMPONENT->z_offset);
//	}
//	else
//	{
//		//COMPONENT->z_offset = -fabs(COMPONENT->z_offset);
//	}
//	//ROS_INFO("Initial ground offset is: %g",COMPONENT->z_offset );
//	COMPONENT->z_offset = 0;
//	events.raiseEvent(Event("/wsm/SensorConnected"));
//	return TaskResult::SUCCESS();
//}

//TaskResult state_STANDBY(string id, const CallContext& context, EventQueue& events){
//	ROS_INFO("WSM_STANDBY");
//	while(pause_time)
//	{
//		ROS_INFO("I'm in pause mode..");
//		PAUSE(1000);
//	}
//	events.raiseEvent(Event("/wsm/Resume",context));
//	return TaskResult::SUCCESS();
//}

//TaskResult state_READY(string id, const CallContext& context, EventQueue& events){
//
//	ROS_INFO("WSM At Ready");
//
//	while(1)
//	{
//		if(events.isTerminated() || !ros::ok()){			/* checks whether the line is empty, or node failed */
//			ROS_INFO("STOPPED");
//			return TaskResult::TERMINATED();
//		}
//
//		while(COMPONENT->cur_mission == NULL){
//			PAUSE(10000);
//			sleep(5);
//		//	ROS_INFO("No new Task");
//		}
//		while(COMPONENT->cur_mission->Get_status() == "active"){
//		//	COMPONENT->cur_mission->debug();
//			COMPONENT->cur_mission->publish_step_diag(1,0);
//			COMPONENT->cur_mission->execute_next_step();
//			COMPONENT->cur_mission->Update_step();
//		}
//		if(COMPONENT->cur_mission->Get_status() == "complete")
//		{
//			events.raiseEvent(Event("/CompleteTask"));
//			ROS_INFO("Mission complete");
//			delete COMPONENT->cur_mission ;
//			COMPONENT->cur_mission = NULL;
//		}
//    }
//
//	return TaskResult::SUCCESS();
//}

class AsyncTask {
protected:
	boost::thread run_thread;
	ComponentMain* comp_ptr;
	Processor * processor_ptr;
	std::string context;
	boost::mutex m;
public:
	AsyncTask(ComponentMain* comp, Processor * processor, std::string event_context) :
			comp_ptr(comp), processor_ptr(processor),  context(event_context) {}

	AsyncTask* start(){
		run_thread = boost::thread(boost::bind(&AsyncTask::start_run, this));
		boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		return this;
	}

	virtual void run() {
		//		diagnostic_msgs::DiagnosticStatus status;
		//		comp_ptr->publishDiagnostic(status);
				ROS_INFO("WSM OFF");
	}

	void start_run() {
		boost::mutex::scoped_lock l(m);
		try
		{
//			cout<<"[d][wsm::AsyncTask]running"<<endl;
			run();
		}
		catch (boost::thread_interrupted& thi_ex) {
//			cout<<"[e][wsm::AsyncTask] thread interrupt signal"<<endl;
		}
		catch (...) {
			ROS_ERROR("WSM::AsyncTask --- Unknown Exception");
//			cout<<"[e][wsm::AsyncTask] unknown exception"<<endl;
		}
	}

	void pause(int millisec) {
		int msI = (millisec / 100), msR = (millisec % 100);
		for (int si = 0; si < msI and not comp_ptr->isClosed(); si++)
			boost::this_thread::sleep(boost::posix_time::millisec(100));
		if (msR > 0 and not comp_ptr->isClosed())
			boost::this_thread::sleep(boost::posix_time::millisec(msR));
	}

	void offTask() {
//		diagnostic_msgs::DiagnosticStatus status;
//		comp_ptr->publishDiagnostic(status);
		ROS_INFO("WSM OFF");
	}

	virtual ~AsyncTask() {
		run_thread.interrupt();
		run_thread.join();
		comp_ptr = NULL;
		processor_ptr = NULL;
	}
};

class TaskInit: public AsyncTask {
public:
	TaskInit(ComponentMain* comp, Processor * processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {}

	virtual void run() {
//		while (!boost::this_thread::interruption_requested() and ros::ok()
//				and comp_ptr->receivedLocation == NULL) {
//			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
//		}

//		pause(10000);
		ROS_INFO("WSM at Init");

		while (comp_ptr->receivedLocation == NULL)
			;
		comp_ptr->z_offset = comp_ptr->receivedLocation->pose.pose.position.z;
		if (comp_ptr->z_offset < 0) {
			comp_ptr->z_offset = fabs(comp_ptr->z_offset);
		} else {
//			comp_ptr->z_offset = fabs(comp_ptr->z_offset);
		}
//		ROS_INFO("Initial ground offset is: %g", comp_ptr->z_offset);
		comp_ptr->z_offset = 0;

		RAISE("/wsm/SensorConnected");
//		RAISE("/SensorConnected");
	}

	virtual ~TaskInit() {}
};

class TaskReady: public AsyncTask {
public:
	TaskReady(ComponentMain* comp, Processor * processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {}

	virtual void run() {
		ROS_INFO("WSM At Ready");

		while (1) {
			if (comp_ptr->isClosed() || !ros::ok())
				ROS_INFO("STOPPED");

			while (comp_ptr->cur_mission == NULL) {
				pause(10000);
				sleep(5);
//				ROS_INFO("No new Task");
			}

			while (comp_ptr->cur_mission->Get_status() == "active") {
//				comp_ptr->cur_mission->debug();
				comp_ptr->cur_mission->publish_step_diag(1, 0);
				comp_ptr->cur_mission->execute_next_step();
				comp_ptr->cur_mission->Update_step();
			}

			if (comp_ptr->cur_mission->Get_status() == "complete") {
//				comp_ptr->rise_taskCompleted();
				processor_ptr->bus_events << cognitao::bus::Event("/CompleteTask");
				ROS_INFO("Mission complete");
				DELETE(comp_ptr->cur_mission);
			}
		}
	}

	virtual ~TaskReady() {}
};

class TaskStandby: public AsyncTask {
public:
	TaskStandby(ComponentMain* comp, Processor * processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {}

	virtual void run() {
//		while (!boost::this_thread::interruption_requested() and ros::ok()) {
//			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//		}

		ROS_INFO("WSM at Standby");
		while (pause_time.get_pause()) {
			ROS_INFO("I'm in pause mode..");
			pause(1000);
		}

		RAISE("/wsm/Resume");
//		RAISE("/Resume");
	}

	virtual ~TaskStandby() {}
};

AsyncTask* task_ptr;

void process_machine(cognitao::machine::Machine & machine,
		Processor & processor, ComponentMain& component) {
	while (processor.empty() == false) {
		cognitao::machine::Event e_poped = processor.pop();
//		cout << "       PROCESS: " << e_poped.str() << endl;
		;
		cognitao::machine::Events p_events;
		machine = machine->process(e_poped, p_events);
		processor.insert(p_events);

		static const cognitao::machine::Event event_about_entry_to_state(
				"task_report?enter");
		if (event_about_entry_to_state.matches(e_poped)) {
			size_t context_size = e_poped.context().size();
			string current_event_context = e_poped.context().str();
			if (context_size > 1) {
				std::string current_task = e_poped.context()[context_size - 2];
//				ROS_INFO_STREAM(
//						" Current event context: " << current_event_context);
//				if (current_task == "off" || current_task == "init" || current_task == "ready" || current_task == "standby")
//					task_ptr->assign(current_event_context, current_task);
				RESET("off", AsyncTask(&component, &processor, current_event_context))
				RESET("init", TaskInit(&component, &processor, current_event_context))
				RESET("ready", TaskReady(&component, &processor, current_event_context))
				RESET("standby", TaskStandby(&component, &processor, current_event_context))
			}
		}

	}
}

void runComponent(int argc, char** argv, ComponentMain& component) {

	ros::NodeHandle node;
	Global_comp = &component;

	ros::Subscriber jointstatesSub = node.subscribe<sensor_msgs::JointState>(
			"/Sahar/joint_states", 100, &JointStatesCallback);
//	ros::Subscriber PauseMission = node.subscribe<std_msgs::String>("/decision_making/events" , 100 , &pauseCallback);

//	cognitao::bus::RosEventQueue events(node, NULL, 1000,
//			"/robil/event_bus/events");
	cognitao::bus::RosEventQueue events(node, NULL, 1000);

	component.set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:wsm}/src/xml/wsm.xml\"/>" << endl
			<< "		<root>wsm</root>" << endl << "	</machines>" << endl
			<< "</tao>" << endl;

	cognitao::machine::Context context("wsm");
	cognitao::io::parser::xml::XMLParser parser;
	cognitao::io::parser::core::MachinesCollection machines;
	try {
		machines = parser.parse(mission_description_stream, context.str());
	} catch (const cognitao::io::parser::core::ParsingError& error) {
		std::cerr << "ParsingError:" << endl << error.message << endl;
		return;
	}

	cognitao::io::compiler::Compiler compiler;
	Processor processor(events);
	compiler.add_builder(
			cognitao::io::compiler::MachineBuilder::Ptr(
					new cognitao::io::compiler::fsm::FsmBuilder(processor)));
	compiler.add_builder(
			cognitao::io::compiler::MachineBuilder::Ptr(
					new cognitao::io::compiler::ftt::FttBuilder(processor)));

	cognitao::io::compiler::CompilationObjectsCollector collector;
	cognitao::io::compiler::CompiledMachine ready_machine;
	try {
		ready_machine = compiler.compile(machines, collector);
	} catch (const cognitao::io::compiler::CompilerError& error) {
		std::cerr << "CompilerError:" << endl << error.message << endl;
		return;
	}

	cout << endl << endl;
//	task_ptr = new AsyncTask(&component, &processor);
	cognitao::machine::Events p_events;
	cognitao::machine::Machine current_machine =
			ready_machine->machine->start_instance(context, p_events);
	processor.insert(p_events);
	process_machine(current_machine, processor, component);

	boost::posix_time::time_duration max_wait_duration(0, 0, 5, 0);
	bool is_timeout = false;
	cognitao::bus::Event event;
	while (events.wait_and_pop_timed(event, max_wait_duration, is_timeout)
			or ros::ok()) {
		if (is_timeout) {
//			cout << "event bus timeout" << endl;
			continue;
		}
//		cout << "GET: " << event << endl;
//		if (event.context().str().find(context.str()) != 0) {
//			cout << "\033[1;31m SKIP event from other node \033[0m\n";
//			continue;
//		}
		pause_checker(event, Global_comp);
		processor.send_no_pub(event);
		process_machine(current_machine, processor, component);
	}

//	delete(task_ptr);

	return;

//	ros_decision_making_init(argc, argv);
//	RosEventQueue events;
//	CallContext context;
//	context.createParameters(new Params(&component));
//	//events.async_spin();
//
//	LocalTasks::registration("OFF",state_OFF);
//	LocalTasks::registration("INIT",state_INIT);
//	LocalTasks::registration("READY",state_READY);
//	LocalTasks::registration("STANDBY",state_STANDBY);
////	event_queue = &events ;
//
//	//ROS_INFO("Starting wsm (WorkSequnceManager)...");
//	//ROS_INFO("WSM AT FSM1");
//	Fsmwsm(&context, &events);
//	//ROS_INFO("WSM AT FSM2");
}

