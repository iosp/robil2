/*
 * EventsHub.cpp
 *
 *  Created on: Jan 7, 2016
 *      Author: dan
 */


#include <ros/ros.h>
#include <vector>
#include <sstream>
#include <set>
#include <map>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <cognitao/bus/RosEventQueue.h>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>


using namespace std;
using namespace boost;
using namespace posix_time;

const string f_source_filter = "--source_filter";
const string f_event_filter = "--event_filter";
const string f_tag_filter = "--tag_filter";
const string f_tag = "--tag";
const string f_input_topic = "--input";
const string f_output_topic = "--output";
const string f_check_rate = "--check_rate";
const string f_end_of_list = "--end_of_list";
const string f_verb = "--verb";

inline
set<string> flags_init()
{
	set<string> f;
	f.insert(f_source_filter);
	f.insert(f_event_filter);
	f.insert(f_tag_filter);
	f.insert(f_tag);
	f.insert(f_input_topic);
	f.insert(f_output_topic);
	f.insert(f_check_rate);
	f.insert(f_end_of_list);
	f.insert(f_verb);
	return f;
}
const set<string> flags = flags_init();

struct Filter{
	bool positive;
	string expression;
	Filter( bool pos, string exp ):positive(pos), expression(exp) {}
};

vector<Filter> source_filters;
vector<Filter> event_filters;
vector<Filter> tag_filters;
vector<Filter> tag;
vector<Filter> inputs;
vector<Filter> outputs;
float check_rate = 1;
bool verb = false;
typedef std::map<string,ros::Subscriber> Subs;
Subs subscribers;
typedef std::map<string,ros::Publisher > Pubs;
Pubs publishers ;

size_t search_flag( vector<string>& args, string flag , int start = 0)
{
	for( size_t i=start; i<args.size(); i++ )
		if( args[i] == flag ) return i;
	return args.size();
}
size_t search_next_flag( vector<string>& args, int start = 0 )
{
	for( size_t i=start; i<args.size(); i++ )
		if( flags.find( args[i] ) != flags.end() ) return i;
	return args.size();
}

void parse_arguments(int a, char** aa)
{
	vector<string> args;
	for(int i=1;i<a;i++) args.push_back(string(aa[i]));

#define FILL_FILTER(f_source_filter, source_filters)\
	{size_t s = search_flag(args, f_source_filter);\
	size_t e = search_next_flag(args, s+1);\
	if( s!=args.size() )\
	{\
		for(size_t i=s+1; i<e; i++)\
		{\
			if( args[i].empty() ) continue;\
			bool p = args[i][0] == '+';\
			bool n = args[i][0] == '-';\
			Filter f( !n , (p or n) ? args[i].substr(1) : args[i] );\
			source_filters.push_back(f);\
		}\
	}}

	FILL_FILTER(f_source_filter	, source_filters)
	FILL_FILTER(f_event_filter	, event_filters)
	FILL_FILTER(f_tag_filter	, tag_filters)
	FILL_FILTER(f_tag	, tag)
	FILL_FILTER(f_input_topic	, inputs)
	FILL_FILTER(f_output_topic	, outputs)

#undef FILL_FILTER
	{size_t s = search_flag(args, f_check_rate);
		if( s < args.size()-1 )
		{
			stringstream ss; ss<<args[s+1];
			ss >> check_rate;
		}
	}

	verb = search_flag(args, f_verb)<args.size();

	size_t s = search_flag(args, "--help");
	if(s<args.size())
	{
		cout<<"Retranslats events from input topics to output topics"<<endl;
		cout<<"Syntax: "<<aa[0]<<" "
				<<f_input_topic<<" [INPUT PARAMS] "
				<<f_output_topic<<" [OUTPUT PARAMS] "
				<<f_check_rate<<" RATE_IN_HZ "
				<<f_source_filter<<" [LIST_OF_REG_EX] "
				<<f_event_filter<<" [LIST_OF_REG_EX] "
				<<f_tag_filter<<" [LIST_OF_REG_EX] "
				<<f_tag<<" [LIST_OF_NEW_TAGS] "
		<<endl;
		cout<<"LIST OF REG EX"<<endl;
		cout<<"   "<<"[+-][RegEx1] [+-][RegEx2] ... . For example: +.*a.* -.*b.* : search all words contains 'a', but does not contain 'b'"<<endl;
		cout<<"INPUT or OUTPUT PARAMS"<<endl;
		cout<<"   "<<"OR detect LIST_OF_REG_EX, OR LIST_OF_TOPICS"<<endl;
		exit(0);
	}
}

void print_arguments()
{
	std::cout<<ros::this_node::getName()<<endl;
	std::cout<<"Source Filter:"<<endl;
	for(size_t i=0;i<source_filters.size();i++)
		std::cout<<"   "<< (source_filters[i].positive==false ? "- ": "+ ") << source_filters[i].expression <<std::endl;
	std::cout<<"Event Filter:"<<endl;
	for(size_t i=0;i<event_filters.size();i++)
		std::cout<<"   "<< (event_filters[i].positive==false ? "- ": "+ ") << event_filters[i].expression <<std::endl;
	std::cout<<"Tag Filter:"<<endl;
	for(size_t i=0;i<tag_filters.size();i++)
		std::cout<<"   "<< (tag_filters[i].positive==false ? "- ": "+ ") << tag_filters[i].expression <<std::endl;
	std::cout<<"Add tags:"<<endl;
	for(size_t i=0;i<tag.size();i++)
		std::cout<<"   "<< (tag[i].positive==false ? "- ": "+ ") << tag[i].expression <<std::endl;
	std::cout<<"Input topics:"<<endl;
	for(size_t i=0;i<inputs.size();i++)
		std::cout<<"   "<< (inputs[i].positive==false ? "- ": "+ ") << inputs[i].expression <<std::endl;
	std::cout<<"Output topics:"<<endl;
	for(size_t i=0;i<outputs.size();i++)
		std::cout<<"   "<< (outputs[i].positive==false ? "- ": "+ ") << outputs[i].expression <<std::endl;
	cout<<"Rate: "<<check_rate<<"Hz"<<endl;
	cout<<"Verb: "<<(verb?"true":"false")<<endl;
}

bool test( string subject, vector<Filter>& filters, size_t filters_offset )
{
	bool res = true;
	if(verb) std::cout<<"TEST: "<<subject<<" ? ["<<ros::this_node::getName()<<"]"<<endl;
	for(size_t i=filters_offset;i<filters.size();i++)
	{
		if(verb) std::cout<<"     "<<filters[i].expression<<"   : "<<(filters[i].positive?"+ ":"- ");
		regex expression(filters[i].expression);
		cmatch what;
		if( filters[i].positive )
		{
			res = res and boost::regex_match(subject.c_str(), what, expression);
			if(verb) cout<<"pos="<<(res?"true":"false");
		}
		else
		{
			res = res and not boost::regex_match(subject.c_str(), what, expression);
			if(verb) cout<<"neg="<<(res?"true":"false");
		}
		if(verb) cout<<endl;
	}
	return res;
}
bool test( string _regex, string _target )
{
	regex exp(_regex);
	cmatch what;
	return boost::regex_match(_target.c_str(), what, exp);
}
bool test_message( const cognitao::bus::message::Event& msg )
{
	bool test_result = test(msg.source, source_filters, 0) and test(msg.event, event_filters, 0);
	for(size_t i=0;i<msg.tags.size() and test_result;i++)
	{
		test_result = test_result and test(msg.tags[i], tag_filters, 0);
	}
	if(verb) std::cout<<"Test result is "<<(test_result?"true => transfer event":"false => drop out event")<<endl;
	return test_result;
}

set<string> get_input_topics()
{
	if( inputs.empty() ) return set<string>();

	if( inputs[0].expression != "detect" )
	{
		set<string> vec;
		for(size_t i=0;i<inputs.size();i++)
			if(inputs[i].positive) vec.insert(inputs[i].expression);
		return vec;
	}

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	set<string> events_topics;

	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
		const ros::master::TopicInfo& info = *it;
		if( "events_bus/Msg_Event" != info.datatype ) continue;
		if(test(info.name, inputs, 1)) events_topics.insert( info.name );
	}

	return events_topics;
}
set<string> get_output_topics()
{
	if( outputs.empty() ) return set<string>();

	if( outputs[0].expression != "detect" )
	{
		set<string> vec;
		for(size_t i=0;i<outputs.size();i++)
			if(outputs[i].positive) vec.insert(outputs[i].expression);
		return vec;
	}

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	set<string> events_topics;

	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
		const ros::master::TopicInfo& info = *it;
		if( "events_bus/Msg_Event" != info.datatype ) continue;
		if(test(info.name, outputs, 1)) events_topics.insert( info.name );
	}

	return events_topics;
}

void on_event( const cognitao::bus::message::Event::ConstPtr& msg )
{
	if( not test_message( *msg ) ) return;

	for(map<string,ros::Publisher>::iterator i=publishers.begin();i!=publishers.end();i++)
	{
		cognitao::bus::message::Event::Ptr e( new cognitao::bus::message::Event );
		*e = *msg;
		e->tags.clear();
		for( size_t v=0;v<msg->tags.size();v++ )
		{
			bool ok=true;
			for(size_t t=0;t<tag.size();t++)
			{
				if(tag[t].positive) continue;
				if( test(tag[t].expression, msg->tags[v]) ) ok=false;
				if( not ok ) break;
			}
			if( ok ) e->tags.push_back(msg->tags[v]);
		}
		for(size_t t=0;t<tag.size();t++)
		{
			if(tag[t].positive==false) continue;
			e->tags.push_back(tag[t].expression);
		}
		i->second.publish(e);
	}
}
#define REMOVE_CHECK(X)
int main( int a, char** aa )
{
	parse_arguments(a, aa);
	print_arguments();

	ros::init(a, aa, "event_hub");

	ros::NodeHandle node;

	ros::Rate rate(1000);
	while(ros::ok())
	{
		set<string> inputs_results = get_input_topics();
		BOOST_FOREACH( string input, inputs_results)
		{
			if( subscribers.find(input) == subscribers.end() )
			{
				subscribers[input] = (node.subscribe(input,1000, on_event));
				boost::this_thread::sleep(seconds(1));
				REMOVE_CHECK( if(subscribers[input].getNumPublishers()!=0) )
					std::cout<<"[i] + <-- "<<input<<endl;
				REMOVE_CHECK( else
					subscribers.erase(input); )
			}
		}
		BOOST_FOREACH( Subs::value_type& input, subscribers)
		{
			if( inputs_results.find(input.first) == inputs_results.end() REMOVE_CHECK( or input.second.getNumPublishers()==0 ))
			{
				cout<<"[i] x <-- "<<input.first<<endl;
				subscribers.erase(input.first);
			}
		}

		set<string> outputs_results = get_output_topics();
		BOOST_FOREACH( string output, outputs_results)
		{
			if( publishers.find(output) == publishers.end() )
			{
				publishers[output] = (node.advertise<cognitao::bus::message::Event>(output,1000));
				REMOVE_CHECK( if(publishers[output].getNumSubscribers()!=0) )
					std::cout<<"[i] + --> "<<output<<endl;
				REMOVE_CHECK( else
					publishers.erase(output); )
			}
		}
		BOOST_FOREACH( Pubs::value_type& output, publishers)
		{
			if( outputs_results.find(output.first) == outputs_results.end() REMOVE_CHECK( or output.second.getNumSubscribers()==0 ))
			{
				cout<<"[i] x --> "<<output.first<<endl;
				publishers.erase(output.first);
			}
		}

		int counter=0;
		if(check_rate==0) check_rate=1000;
		while(ros::ok() and ++counter <= 1000/check_rate )
		{
			ros::spinOnce();
			rate.sleep();
		}
	}
	return 0;
}

