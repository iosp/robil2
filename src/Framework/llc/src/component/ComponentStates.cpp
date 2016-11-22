#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include "aux_functions.h"
#include <bondcpp/bond.h>
#include <llc/ControlParamsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <robil_msgs/GpsSpeed.h>
#include "ComponentStates.h"
#include <math.h>

using namespace std;

#include "ComponentStates.h"

#define DELETE(X) if(X){delete X; X=NULL;}
#define RESET(X,Y) if(current_task == X) { \
					ROS_WARN_STREAM(" Current LLC task: " << current_task); \
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

#define DT 0.01
#define LENGTH_OF_RECORD_IN_SECONDS 3
#define LENGTH_OF_RECORD_IN_FRAMES (1/DT)*LENGTH_OF_RECORD_IN_SECONDS
#define SIZE_OF_WPD_INTEGRAL 3
static double P_linear, D_linear, I_linear; 				/* PID constants of linear x */
static double P_angular, D_angular, I_angular;		/* PID constants of angular z */
static double linearNormalizer;
double sum_linear;
double sum_angular;
ros::Publisher Throttle_rate_pub;
ros::Publisher Steering_rate_pub;

double errorAngularArray[1000];
int indexOf_errorAngularArray;
double errorLinearArray[1000];
int indexOf_errorLinearArray;
double wpdCmdLinearArray[SIZE_OF_WPD_INTEGRAL];
int indexOf_wpdCmdLinearArray;
double wpdCmdAngularArray[SIZE_OF_WPD_INTEGRAL];
int indexOf_wpdCmdAngularArray;
double lastLinearError;
double lastAngularError;
double LocVelLinearX;
double LocVelLinearY;
double LocVelAngularZ;
double sumOfWpdSpeedLinear;
double sumOfWpdSpeedAngular;
double WpdSpeedLinear;
double WpdSpeedAngular;
double currentYaw;
double currentVelocity;
double wpdSpeedTimeInMilli;
double linearFactor;
double angularFactor;

void dynamic_Reconfiguration_callback(llc::ControlParamsConfig &config, uint32_t level) {

	P_linear=config.linearVelocity_P;
	I_linear=config.linearVelocity_I;
	D_linear=config.linearVelocity_D;
	P_angular=config.angularVelocity_P;
	I_angular=config.angularVelocity_I;
	D_angular=config.angularVelocity_D;

	linearFactor=config.linearFactor;
	angularFactor=config.angularFactor;
}

void cb_currentYaw(geometry_msgs::PoseWithCovarianceStamped msg)
{
  double roll, pitch, yaw = 0;
        tf::Quaternion q(  msg.pose.pose.orientation.x ,  msg.pose.pose.orientation.y,   msg.pose.pose.orientation.z,   msg.pose.pose.orientation.w);
          tf::Matrix3x3 rot_mat(q);
          rot_mat.getEulerYPR(yaw,pitch,roll);

          currentYaw = yaw;
}

double calcIntegral_linearError(double currError)
{
  sum_linear -= errorLinearArray[indexOf_errorLinearArray];
  sum_linear += currError;

  errorLinearArray[indexOf_errorLinearArray] = currError;

  if(++indexOf_errorLinearArray >= LENGTH_OF_RECORD_IN_FRAMES)
          indexOf_errorLinearArray = 0;

  return sum_linear*DT;
}

double calcDiferencial_linearError(double currError)
{
  double diff = currError - lastLinearError;
  double result = diff / DT;
  lastLinearError = currError;
  return result;
}

double calcIntegral_angularError(double currError)
{
  sum_angular -= errorAngularArray[indexOf_errorAngularArray];
  sum_angular += currError;

  errorAngularArray[indexOf_errorAngularArray] = currError;
  indexOf_errorAngularArray++;

  if(indexOf_errorAngularArray >= LENGTH_OF_RECORD_IN_FRAMES)
          indexOf_errorAngularArray = 0;

  return sum_angular*DT;
}

double calcDiferencial_angularError(double currError)
{
  double diff = currError - lastAngularError;
  double result = diff / DT;
  lastAngularError = currError;
  return result;
}

void cb_LocVelpcityUpdate(geometry_msgs::TwistStamped msg)
{
  LocVelLinearX = msg.twist.linear.x;
  LocVelLinearY = msg.twist.linear.y;

  LocVelAngularZ = msg.twist.angular.z;

  double V_normal = sqrt((LocVelLinearX * LocVelLinearX) + (LocVelLinearY * LocVelLinearY));
  if (V_normal > 0.01)
    {
      double x_normal = LocVelLinearX/V_normal;
      double y_noraml = LocVelLinearY/V_normal;
      double theta = atan2(y_noraml, x_normal);
      if(abs(theta - currentYaw) - 3.0/8.0*M_PI < 0.01)
        {
          currentVelocity = V_normal;
        }
      else if (abs(theta - currentYaw) - 5.0/8.0*M_PI < 0.01)
        {
          currentVelocity = 0;
        }
      else
        currentVelocity = -V_normal;
    }
}

void cb_WpdSpeed(geometry_msgs::TwistStamped msg)
{
  wpdSpeedTimeInMilli = ros::Time::now().toSec()*1000; // toSec() return seconds.milliSecconds

  sumOfWpdSpeedLinear -= wpdCmdLinearArray[indexOf_wpdCmdLinearArray];
  sumOfWpdSpeedLinear += msg.twist.linear.x;
  sumOfWpdSpeedAngular -= wpdCmdAngularArray[indexOf_wpdCmdAngularArray];
  sumOfWpdSpeedAngular += msg.twist.angular.z;

  wpdCmdLinearArray[indexOf_wpdCmdLinearArray] = msg.twist.linear.x;
  wpdCmdAngularArray[indexOf_wpdCmdAngularArray] = msg.twist.angular.z;
  if(++indexOf_wpdCmdLinearArray >= SIZE_OF_WPD_INTEGRAL)
    indexOf_wpdCmdLinearArray = 0;
  if(++indexOf_wpdCmdAngularArray >= SIZE_OF_WPD_INTEGRAL)
    indexOf_wpdCmdAngularArray = 0;

  WpdSpeedLinear = sumOfWpdSpeedLinear/SIZE_OF_WPD_INTEGRAL;
  WpdSpeedAngular = sumOfWpdSpeedAngular/SIZE_OF_WPD_INTEGRAL;
}

void pubThrottleAndSteering()
{
    double RosTimeNowInMilli = ros::Time::now().toSec()*1000;  // toSec() return seconds.milliSecconds

    //if there is no WPD command as long as  500ms, give the zero command
    if(wpdSpeedTimeInMilli + 500 - RosTimeNowInMilli <= 0)
      {
        WpdSpeedLinear = 0;
        WpdSpeedAngular = 0;
      }

    double linearError = (linearFactor*WpdSpeedLinear) - currentVelocity;
    double linearEffortCMD = P_linear * linearError + I_linear* calcIntegral_linearError(linearError)+ D_linear * calcDiferencial_linearError(linearError);

    std_msgs::Float64 msglinearEffortCMD;
    msglinearEffortCMD.data = linearEffortCMD;
    Throttle_rate_pub.publish(msglinearEffortCMD);

    double angularError = (angularFactor * WpdSpeedAngular) - LocVelAngularZ;
    double angularEffortCMD = P_angular * angularError + I_angular* calcIntegral_angularError(angularError) + D_angular * calcDiferencial_angularError(angularError);

    if(linearEffortCMD < 0)
      angularEffortCMD = -angularEffortCMD;
    std_msgs::Float64 msgAngularEffortCMD;
    msgAngularEffortCMD.data = angularEffortCMD;
    Steering_rate_pub.publish(msgAngularEffortCMD);
}

class AsyncTask {
protected:
	boost::thread run_thread;
	ComponentMain* comp_ptr;
	Processor* processor_ptr;
	std::string context;
	boost::mutex m;
public:
	AsyncTask(ComponentMain* comp, Processor * processor, std::string context) :
			comp_ptr(comp), processor_ptr(processor), context(context) {}

	virtual void run() {
		ROS_INFO("LLC OFF");
		//diagnostic_msgs::DiagnosticStatus status;
		//comp_ptr->publishDiagnostic(status);
	}

	AsyncTask* start(){
		run_thread = boost::thread(boost::bind(&AsyncTask::start_run, this));
		boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		return this;
	}

	void start_run() {
		boost::mutex::scoped_lock l(m);
		try
		{
			run();
		}
		catch (boost::thread_interrupted& thi_ex) {
		}
		catch (...) {
			ROS_ERROR("LLC::AsyncTask --- Unknown Exception");
		}
	}

	void pause(int millisec) {
		int msI = (millisec / 100), msR = (millisec % 100);
		for (int si = 0; si < msI and not comp_ptr->isClosed(); si++)
			boost::this_thread::sleep(boost::posix_time::millisec(100));
		if (msR > 0 and not comp_ptr->isClosed())
			boost::this_thread::sleep(boost::posix_time::millisec(msR));
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
	TaskInit(ComponentMain* comp, Processor* processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {}

	virtual void run() {
		ROS_INFO("LLC Init");
		sum_linear = 0;
		sum_angular = 0;
		LocVelLinearX = 0;
		LocVelLinearY = 0;
		LocVelAngularZ = 0;
		WpdSpeedLinear = 0;
		WpdSpeedAngular = 0;
		indexOf_errorAngularArray = 0;
		indexOf_errorLinearArray = 0;
		indexOf_wpdCmdAngularArray = 0;
		indexOf_wpdCmdLinearArray = 0;
		sumOfWpdSpeedLinear = 0;
		sumOfWpdSpeedAngular = 0;
		currentYaw = 0;
		currentVelocity = 0;
		linearNormalizer = 1;
		wpdSpeedTimeInMilli = 0;

		for(int i = 0 ; i < 1000 ; i++)
		{
		    errorLinearArray[i] = 0;
		    errorAngularArray[i] = 0;
		}
		for(int i = 0 ; i < SIZE_OF_WPD_INTEGRAL ; i++)
		{
		    wpdCmdLinearArray[i] = 0;
		    wpdCmdAngularArray[i] = 0;
		}

		RAISE("/llc/EndOfInit");
	}

	virtual ~TaskInit() {}
};

class TaskReady: public AsyncTask {
public:
	TaskReady(ComponentMain* comp, Processor* processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {}

	virtual void run() {
		ros::NodeHandle n;

	    Throttle_rate_pub = n.advertise<std_msgs::Float64>("/LLC/EFFORTS/Throttle", 100);
	    Steering_rate_pub = n.advertise<std_msgs::Float64>("/LLC/EFFORTS/Steering", 100);

	    ros::Subscriber locVel= n.subscribe("/LOC/Velocity" , 10, cb_LocVelpcityUpdate);
	    ros::Subscriber locPose= n.subscribe("/LOC/Pose" , 10, cb_currentYaw);
	    ros::Subscriber wpdSpeed = n.subscribe("/WPD/Speed" , 10, cb_WpdSpeed);

	    ROS_INFO("LLC Ready");

	    ros::Rate rate(100);
	    while(ros::ok())
	    {
	    	pubThrottleAndSteering();
	    	rate.sleep();
	    	ros::spinOnce();
	    }
	}

	virtual ~TaskReady() {}
};

class TaskStandby: public AsyncTask {
public:
	TaskStandby(ComponentMain* comp, Processor* processor,
			std::string current_context) :
			AsyncTask(comp, processor, current_context) {}

	virtual void run() {
		ROS_INFO("LLC Standby");
	}

	virtual ~TaskStandby() {}
};

AsyncTask* task_ptr;

void process_machine(cognitao::machine::Machine & machine,
		Processor & processor, ComponentMain& component) {
	while (processor.empty() == false) {
		cognitao::machine::Event e_poped = processor.pop();
//		cout << "       PROCESS: " << e_poped.str() << endl;
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
//				if (current_task == "off" || current_task == "init" || current_task == "ready" || current_task == "standby")
//					task_ptr->assign(current_event_context, current_task);
//				ROS_WARN_STREAM(" Current task: " << current_task);
//				ROS_INFO_STREAM(" Current event context: " << current_event_context);
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
//	cognitao::bus::RosEventQueue events(node, NULL, 1000,
//			"/robil/event_bus/events");
	cognitao::bus::RosEventQueue events(node, NULL, 1000);

	component.set_events(&events);

	std::stringstream mission_description_stream;
	mission_description_stream << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
			<< endl << "<tao>" << endl << "	<machines>" << endl
			<< "		<machine file=\"${rospack:llc}/src/xml/llc.xml\"/>" << endl
			<< "		<root>llc</root>" << endl << "	</machines>" << endl
			<< "</tao>" << endl;

	cognitao::machine::Context context("llc");
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
//			//cout << "\033[1;31m SKIP event from other node \033[0m\n";
//			continue;
//		}
		processor.send_no_pub(event);
		process_machine(current_machine, processor, component);
	}

	return;

//	ros::init(argc, argv, "llc");
//	ros_decision_making_init(argc, argv);
//	RosEventQueue events;
//	CallContext context;
//	context.createParameters(new Params(&component));
//	//events.async_spin();
//	LocalTasks::registration("OFF",state_OFF);
//	LocalTasks::registration("INIT",state_INIT);
//	LocalTasks::registration("READY",state_READY);
//	LocalTasks::registration("STANDBY",state_STANDBY);
//
//    dynamic_reconfigure::Server<llc::ControlParamsConfig> server;
//    dynamic_reconfigure::Server<llc::ControlParamsConfig>::CallbackType f;
//
//    f = boost::bind(&dynamic_Reconfiguration_callback, _1, _2);
//    server.setCallback(f);
//
//	ROS_INFO("Starting llc...");
//	Fsmllc(&context, &events);

}
