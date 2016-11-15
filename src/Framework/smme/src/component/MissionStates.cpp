#include <iostream>
#include <ros/ros.h>
//#include <decision_making/SynchCout.h>
//#include <decision_making/BT.h>
//#include <decision_making/FSM.h>
//#include <decision_making/ROSTask.h>
//#include <decision_making/DecisionMaking.h>
using namespace std;
//using namespace decision_making;
#include "ComponentStates.h"
#include "MissionManager.h"

//class MissionParams: public CallContextParameters{
//public:
//	ComponentMain* comp;
//	string mission_id;
//	MissionParams():comp(0), mission_id(""){struct MissionParametersConstructorEmpty{}; throw MissionParametersConstructorEmpty();}
//	MissionParams(ComponentMain* comp, string mission_id)
//		:comp(comp), mission_id(mission_id)
//	{
//
//	}
//	std::string str()const{return "";}
//};

#define M_PREF string("smme/mission")
#define T_PREF string("smme/task")
#define DELETE(X) if(X){delete X; X=NULL;}
#define RESET(X,Y) if(current_task == X) { \
					ROS_WARN_STREAM(" Current mission: " << current_task); \
					DELETE(mission_ptr) \
					mission_ptr = new Y; \
					mission_ptr->start(); \
					continue;}
#define EVENT(E,C) \
		cognitao::bus::Event( \
				cognitao::bus::Event::name_t(E), \
				cognitao::bus::Event::channel_t(""), \
				cognitao::bus::Event::context_t(C))
#define RAISE(X,C) processor_ptr->bus_events << EVENT(X,C)
#define MISSION_ID MID_PREF(FSM_CONTEXT.parameters<MissionParams>().mission_id)

//FSM(MissionActive)
//{
//	FSM_STATES
//	{
//		MissionSpooling,
//		MissionPaused,
//		MissionAborted,
//		MissionFinished,
//	}
//	FSM_START(MissionSpooling);
//	FSM_BGN
//	{
//		FSM_STATE(MissionSpooling)
//		{
//			FSM_RAISE(MISSION_ID+"/StartTask")
//			FSM_RAISE(MISSION_ID+"/ResumeTask")
//			FSM_CALL_TASK(MissionSpooling)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(MISSION_ID+"/CompleteMission", FSM_NEXT(MissionFinished));
//				FSM_ON_EVENT(MISSION_ID+"/AbortMission", FSM_NEXT(MissionAborted));
//				FSM_ON_EVENT(MISSION_ID+"/PauseMission", FSM_NEXT(MissionPaused));
//			}
//		}
//		FSM_STATE(MissionPaused)
//		{
//			//FSM_RAISE(MISSION_ID+"/PauseTask")
//			FSM_RAISE(MISSION_ID+"/StopTask")
//			FSM_CALL_TASK(MissionPaused)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(MISSION_ID+"/CompleteMission", FSM_NEXT(MissionFinished));
//				FSM_ON_EVENT(MISSION_ID+"/AbortMission", FSM_NEXT(MissionAborted));
//				FSM_ON_EVENT(MISSION_ID+"/ResumeMission", FSM_NEXT(MissionSpooling));
//			}
//		}
//		FSM_STATE(MissionAborted)
//		{
//			FSM_RAISE(MISSION_ID+"/StopTask")
//			FSM_CALL_TASK(MissionAborted)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(MISSION_ID+"/StartMission", FSM_NEXT(MissionSpooling));
//			}
//		}
//		FSM_STATE(MissionFinished)
//		{
//			FSM_RAISE(MISSION_ID+"/StopTask")
//			FSM_CALL_TASK(MissionFinished)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT(MISSION_ID+"/StartMission", FSM_NEXT(MissionSpooling));
//			}
//		}
//	}
//	FSM_END
//}
//
//FSM(Mission)
//{
//	FSM_STATES
//	{
//		MissionUnloaded,
//		MissionPending,
//		MissionActive
//	}
//	FSM_START(MissionPending);
//	call_ctx.pop();
//	FSM_BGN
//	{
//		FSM_STATE(MissionUnloaded)
//		{
//			FSM_RAISE(MISSION_ID+"/StopTask");
//			FSM_CALL_TASK(MissionUnloaded)
//			FSM_TRANSITIONS
//			{
//				FSM_ON_EVENT("MissionUnloaded/Stopped", FSM_STOP("Stopped",TaskResult::SUCCESS()));
//			}
//		}
//		FSM_STATE(MissionPending)
//		{
//			FSM_CALL_TASK(MissionPending)
//			FSM_TRANSITIONS
//			{
//				//NOTE: It's not clear for transition from MissionPending to NoMissionLoaded
//				FSM_ON_EVENT(MISSION_ID+"/DeleteMission", FSM_NEXT(MissionUnloaded));
//				FSM_ON_EVENT("/ClearMissionBuffer", FSM_NEXT(MissionUnloaded));
//
//				FSM_ON_EVENT(MISSION_ID+"/StartMission", FSM_NEXT(MissionActive));
//			}
//		}
//		FSM_STATE(MissionActive)
//		{
//			FSM_CALL_FSM(MissionActive)
//			FSM_TRANSITIONS
//			{
//				//NOTE: It's not clear for transition from MissionActive to NoMissionLoaded
//				FSM_ON_EVENT(MISSION_ID+"/DeleteMission", FSM_NEXT(MissionUnloaded));
//				FSM_ON_EVENT(MISSION_ID+"/Standby", FSM_NEXT(MissionUnloaded));
//				FSM_ON_EVENT(MISSION_ID+"/ClearMissionBuffer", FSM_NEXT(MissionUnloaded));
//			}
//		}
//
//	}
//	FSM_END
//}

//#define PARAMS \
//		std::string mid = context.parameters<MissionParams>().mission_id;\
//		ComponentMain* comp = context.parameters<MissionParams>().comp;


bool extend_events_names(cognitao::bus::Event& e, std::string mid_pref, cognitao::bus::EventQueue* events){
#		define EXTEND(NAME) \
		if(e == cognitao::bus::Event(NAME)){\
			std::string ex=mid_pref+NAME;\
			ROS_INFO("Extend event: "NAME" to %s",ex.c_str());\
			events->rise(cognitao::bus::Event( \
							cognitao::bus::Event::name_t(mid_pref+NAME), \
							cognitao::bus::Event::channel_t(""), \
							cognitao::bus::Event::context_t("smme/task"))); \
			return true;\
		}
		//----------- TASK GLOBAL EVENT -------------
		EXTEND("/StopTask")
		EXTEND("/CompleteTask")
		EXTEND("/AbortTask")
		EXTEND("/PauseTask")
		EXTEND("/ResumeTask")
		//----------- MISSION GLOBAL EVENT ----------
		EXTEND("/CompleteMission")
		EXTEND("/PauseMission")
		EXTEND("/AbortMission")
		EXTEND("/ResumeMission")
		return false;
#		undef EXTEND
}

class AsyncMission {
protected:
	ComponentMain * comp_ptr;
	boost::thread run_thread;
	Processor * processor_ptr;
	std::string context;
	boost::mutex m;

public:

#define MID std::string mid = MM->get_current_mission().mid;	// TODO: check this
#define MM comp_ptr->mission_manager()

	AsyncMission(ComponentMain* comp, Processor * processor, std::string event_context) :
		comp_ptr(comp), processor_ptr(processor),  context(event_context) {
		boost::mutex::scoped_lock l(m);
	}

	AsyncMission* start(){
		run_thread = boost::thread(boost::bind(&AsyncMission::start_run, this));
		boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		return this;
	}

	virtual void run() {
		ROS_INFO("SMME mission at Pending.");
		MID
		MissionManager::MissionID cmid = MM->get_current_mission().mid;
		MM->change_mission(mid);
		MM->mission_state("pending");
		MM->change_mission(cmid);
	}

	void start_run() {
		boost::mutex::scoped_lock l(m);
		try
		{
//			cout<<"[d][smme-mission::AsyncTask]running"<<endl;
			run();
		}
		catch (boost::thread_interrupted& thi_ex) {
//			cout<<"[e][smme-mission::AsyncTask] thread interrupt signal"<<endl;
		}
		catch (...) {
			ROS_ERROR("SMME::AsyncMission --- Unknown Exception");
//			cout<<"[e][smme-mission::AsyncTask] unknown exception"<<endl;
		}
	}

	void pause(int millisecs) {
		int msI = (millisecs / 100), msR = (millisecs % 100);
		for (int si = 0; si < msI and not comp_ptr->events()->is_closed(); si++)
			boost::this_thread::sleep(boost::posix_time::millisec(100));
		if (msR > 0 and not comp_ptr->events()->is_closed())
			boost::this_thread::sleep(boost::posix_time::millisec(msR));
	}

	virtual ~AsyncMission() {
		run_thread.interrupt();
		run_thread.join();
		comp_ptr = NULL;
		processor_ptr = NULL;
	}
};

class AsyncMissionSpooling : public AsyncMission {
public:
	AsyncMissionSpooling(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncMission(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&AsyncMissionSpooling::run, this));
	}

	virtual void run() {
		ROS_INFO("SMME mission at Spooling.");
		MID

		RAISE(mid + "/StartTask", T_PREF);
		RAISE(mid + "/ResumeTask", T_PREF);

		MM->change_mission(mid);
		MM->mission_state("spooling");
		while(comp_ptr->events()->is_closed()==false and ros::ok()){
			cognitao::bus::Event e = comp_ptr->events()->waitEvent();
			extend_events_names(e, mid, comp_ptr->events());
			if(e == cognitao::bus::Event(mid + "/CompleteTask") or e == cognitao::bus::Event(mid +"/StopTask")){
				std::string ex = mid + "/CompleteTask";
				ROS_INFO("Event %s detected. got next or complete mission", ex.c_str());
				if(MM->next_task()){
					this_thread::sleep(milliseconds(100));
					RAISE(mid + "/StartTask", T_PREF);
				}else{
					RAISE(mid + "/CompleteMission", M_PREF);
				}
			}
		}
	}

	virtual ~AsyncMissionSpooling() {}
};

class AsyncMissionPaused : public AsyncMission {
public:
	AsyncMissionPaused(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncMission(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&AsyncMissionPaused::run, this));
	}

	virtual void run() {
		ROS_INFO("SMME mission at Paused.");
		MID
		RAISE(mid + "/PauseTask", T_PREF);

		MM->mission_state("paused");
		while(comp_ptr->events()->is_closed()==false and ros::ok()){
			cognitao::bus::Event e = comp_ptr->events()->waitEvent();
			//extend_events_names(e, MID_PREF(mid), comp_ptr->events());
		}
	}

	virtual ~AsyncMissionPaused() {}
};

class AsyncMissionAborted : public AsyncMission {
public:
	AsyncMissionAborted(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncMission(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&AsyncMissionAborted::run, this));
	}

	virtual void run() {
		ROS_INFO("SMME mission at Aborted.");
		MID
		RAISE(mid + "/AbortTask", T_PREF);

		MM->mission_state("aborted");
	}

	virtual ~AsyncMissionAborted() {}
};

class AsyncMissionFinished : public AsyncMission {
public:
	AsyncMissionFinished(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncMission(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&AsyncMissionFinished::run, this));
	}

	virtual void run() {
		ROS_INFO("SMME mission at Finished.");
		MID
		RAISE(mid + "/CompleteTask", T_PREF(""));

		MM->mission_state("finished");
	}

	virtual ~AsyncMissionFinished() {}
};

class AsyncMissionUnloaded : public AsyncMission {
public:
	AsyncMissionUnloaded(ComponentMain* comp, Processor * processor,
			std::string current_context) : AsyncMission(comp,processor,current_context) {
//		run_thread = boost::thread(boost::bind(&AsyncMissionFinished::run, this));
	}

	virtual void run() {
		ROS_INFO("SMME mission at Unloaded.");
		MID
		RAISE(mid + "/StopTask", T_PREF(""));

		MM->mission_state("finished");
	}

	virtual ~AsyncMissionUnloaded() {}
};

//TaskResult state_MissionUnloaded(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MM->remove(mid);
//	events.raiseEvent(Event("Stopped",context));
//	return TaskResult::SUCCESS();
//}
//TaskResult state_MissionPending(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MissionManager::MissionID cmid = MM->get_current_mission().mid;
//	MM->change_mission(mid);
//	MM->mission_state("pending");
//	MM->change_mission(cmid);
//	return TaskResult::SUCCESS();
//}
//
//TaskResult state_MissionSpooling(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MM->change_mission(mid);
//	MM->mission_state("spooling");
//	while(events.isTerminated()==false and ros::ok()){
//		Event e = events.waitEvent();
//		extend_events_names(e, MID_PREF(mid), events);
//		if(e == Event(MID_PREF(mid)+"/CompleteTask") or e == Event(MID_PREF(mid)+"/StopTask")){
//			std::string ex = MID_PREF(mid)+"/CompleteTask";
//			ROS_INFO("Event %s detected. got next or complete mission", ex.c_str());
//			if( MM->next_task() ){
//				this_thread::sleep(milliseconds(100));
//				events.raiseEvent(MID_PREF(mid)+"/StartTask");
//			}else{
//				events.raiseEvent(MID_PREF(mid)+"/CompleteMission");
//			}
//		}
//	}
//	return TaskResult::SUCCESS();
//}
//TaskResult state_MissionPaused(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MM->mission_state("paused");
//	while(events.isTerminated()==false and ros::ok()){
//		Event e = events.waitEvent();
//		//extend_events_names(e, MID_PREF(mid), events);
//	}
//	return TaskResult::SUCCESS();
//}
//TaskResult state_MissionAborted(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MM->mission_state("aborted");
//	return TaskResult::SUCCESS();
//}
//TaskResult state_MissionFinished(string id, const CallContext& context, EventQueue& events){
//	PARAMS
//	MM->mission_state("finished");
//	return TaskResult::SUCCESS();
//}
//
//
//
//void initMissionTasks(){
//	LocalTasks::registration("MissionUnloaded",state_MissionUnloaded);
//	LocalTasks::registration("MissionPending",state_MissionPending);
//	LocalTasks::registration("MissionSpooling",state_MissionSpooling);
//	LocalTasks::registration("MissionPaused",state_MissionPaused);
//	LocalTasks::registration("MissionAborted",state_MissionAborted);
//	LocalTasks::registration("MissionFinished",state_MissionFinished);
//}

MissionMachine::MissionMachine(ComponentMain* comp,std::string mid)
:
		component(comp),mission_id(mid),task(comp,mid),events_ptr(0)
{
	static int _init=init(comp,mid);
	manager = component->mission_manager();
	manager->start_mission(mid);
}

AsyncMission* mission_ptr;

void process_mission(cognitao::machine::Machine & machine,
		Processor & processor, ComponentMain& component) {
	while (processor.empty() == false) {
		cognitao::machine::Event e_poped = processor.pop();
//		cout << "       PROCESS_MISSION: " << e_poped.str() << endl;
//		;
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
//				if (current_task == "spooling" || current_task == "paused"
//						|| current_task == "aborted"
//						|| current_task == "finished"
//						|| current_task == "pending"
//						|| current_task == "unloaded")
//					mission_ptr->assign(current_event_context, current_task);
//				ROS_WARN_STREAM(" Current mission: " << current_task);
//				ROS_INFO_STREAM(
//						" Current event context: " << current_event_context);
				RESET("pending", AsyncMission(&component, &processor, current_event_context))
				RESET("spooling", AsyncMissionSpooling(&component, &processor, current_event_context))
				RESET("paused", AsyncMissionPaused(&component, &processor, current_event_context))
				RESET("aborted", AsyncMissionAborted(&component, &processor, current_event_context))
				RESET("finished", AsyncMissionFinished(&component, &processor, current_event_context))
				RESET("unloaded", AsyncMissionUnloaded(&component, &processor, current_event_context))
			}
		}
	}
}

void MissionMachine::startMission(ComponentMain* component, std::string mission_id){

	ros::NodeHandle node;
//	cognitao::bus::RosEventQueue events(node, NULL, 1000,
//			"/robil/event_bus/events");
	cognitao::bus::RosEventQueue events(node, NULL, 1000);

	component->set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:smme}/src/xml/smme_mission.xml\" MID=\"" << mission_id << "\" />"
			<< endl << "		<root>mission</root>" << endl << "	</machines>" << endl
			<< "</tao>" << endl;

	cognitao::machine::Context context("smme/mission");
	cognitao::io::parser::xml::XMLParser parser;
	cognitao::io::parser::MachinesCollection machines;
	try {
		machines = parser.parse(mission_description_stream, context.str());
	} catch (const cognitao::io::parser::ParsingError& error) {
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
//	mission_ptr = new AsyncMission(component, &processor);
	cognitao::machine::Events p_events;
	cognitao::machine::Machine current_machine =
			ready_machine->machine->start_instance(context, p_events);
	processor.insert(p_events);
	process_mission(current_machine, processor, *component);

	time_duration max_wait_duration(0, 0, 5, 0);
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
		processor.send_no_pub(event);
		process_mission(current_machine, processor, *component);
	}

//	delete(mission_ptr);

	return;

//	RosEventQueue events;
//	events_ptr = &events;
//	CallContext context;
//	context.push("mission");
//	context.push(mission_id);
//	context.createParameters(new MissionParams(component, mission_id));
//
//	ROS_INFO_STREAM("Starting smme (Mission:"<<mission_id<<")...");
//	FsmMission(&context, &events);
//	ROS_INFO_STREAM("Stop smme (Mission:"<<mission_id<<")");
}

void MissionMachine::stop(){
	task.stop();
	static_cast<cognitao::bus::RosEventQueue*>(events_ptr)->close();
	thread.join_all();
}
