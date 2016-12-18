
#include <cognitao/io/parser/xml/ros.h>

#include <ros/ros.h>

#include <cognitao/io/parser/xml/arguments_substitution.h>
#include <cognitao/io/parser/xml/context_filling.h>
#include <cognitao/io/parser/xml/sections.h>
#include <cognitao/io/parser/xml/subs.h>
#include <cognitao/io/parser/xml/tokens.h>
#include <cognitao/io/parser/xml/xml_args.h>

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace ros{

using namespace sections;
using namespace tokens;
using namespace iostream;

string search_ros_package(string ros_pack_name)
{
	try{
		string result = exec("rospack find "+ros_pack_name);
		if( result.find("[rospack] Error:")!=string::npos ) return "";
		trim(result);
		return result;
	}catch(...){}
	return "";
}

string search_ros_parameter(string ros_param_name)
{
	string parameter;
	using namespace ::ros;
	try{
		bool is_ros_inited = isInitialized() and isStarted();
		if( not is_ros_inited )
		{
			int arg=0;
			char** argv=0;
			::ros::init(arg, argv, "node", ::ros::init_options::AnonymousName);
			::ros::start();
		}

		try{
			parameter = param::param(ros_param_name,string(""));
		}catch(...){}

		if( not is_ros_inited )
		{
			::ros::shutdown();
		}
	}catch(...){}

	return parameter;
}

}}}}}

