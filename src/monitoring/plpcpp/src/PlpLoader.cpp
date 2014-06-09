/*
 * PlpLoader.cpp
 *
 *  Created on: Jun 9, 2014
 *      Author: dan
 */

#include <plpcpp/PlpLoader.h>


#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>

using namespace std;
using namespace boost;



namespace plp{
string create_header(){
	stringstream s_name;
	boost::posix_time::ptime stime = boost::posix_time::microsec_clock::local_time();
	s_name << boost::posix_time::to_iso_string(stime);
	string s = string()+
		"#! type plp\n"
		"#! name plp_script_"+s_name.str()+"\n"
		"#! interval 1\n";
	;
	//cout<<"HEADER: "<<s<<endl;
	return s;
}

PlpLoader::PlpLoader(string filename) {
	_header=create_header();
	load_plp(filename);
}
PlpLoader::PlpLoader(istream& stream) {
	_header=create_header();
	load_plp(stream);
}
PlpLoader::PlpLoader() {
	_header=create_header();
}

PlpLoader::~PlpLoader() {

}

void PlpLoader::load_plp(istream& stream){
	stringstream text;
	while(stream.eof()==false){
		char c; stream.read(&c,1);
		if(c=='\r')continue;
		text<<c;
	}
	_plp_text = text.str();
	parse();
	if(plp_name().empty()){
		throw Exception_PlpHasNoName();
	}
}
void PlpLoader::load_plp(string filename){
	ifstream file(filename.c_str());
	if(not file){
		throw Exception_FileLoadProblem();
	}
	load_plp(file);
}


string PlpLoader::plp_name()const{
	if(_plp_map.find("PLP:Name")==_plp_map.end()) return "";
	return boost::replace_all_copy(_plp_map.at("PLP:Name")," ","_");
}
string PlpLoader::plp_type()const{
	if(_plp_map.find("PLP:Type")==_plp_map.end()) return "";
	return _plp_map.at("PLP:Type");
}
bool PlpLoader::plp_is_repeated()const{
	if(_plp_map.find("Goals:Repeat")==_plp_map.end()) return false;
	return to_upper_copy(_plp_map.at("Goals:Repeat")) == "TRUE";
}

struct KeyValue{
	string key;
	string value;
	string section;
	string subsection;
	string full_key()const{
		stringstream s;
		if(section.empty()==false){
			s<<section<<":";
			if(subsection.empty()==false){
				s<<subsection<<":";
			}
		}
		s<<key;
		return s.str();
	}
};
KeyValue parse(string line){
	trim(line);
	vector<string> kv;
	split(kv, line, boost::is_any_of(":"));
	if(kv.size()<2)kv.push_back("");
	if(kv.size()<2)return KeyValue();
	trim(kv[0]); trim(kv[1]);
	KeyValue r={kv[0],kv[1]};
	return r;
}
void parse(KeyValue& kv, string line){
	KeyValue r = parse(line);
	kv.key=r.key;
	kv.value=r.value;
}

void check_if_plp_section(KeyValue& kv){
	if(kv.key=="PLP"){
		kv.section="PLP";kv.subsection="";kv.key="Name";
	}else
	if(kv.key=="Goal repeat"){
		kv.section="Goals";kv.subsection="";kv.key="Repeat";
	}else
	if(kv.key=="Variables"){
		kv.section="Variables";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Parameters"){
		kv.section="Parameters";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Preconditions"){
		kv.section="Preconditions";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Concurrent conditions"){
		kv.section="Concurrent conditions";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Concurrent modules"){
		kv.section="Concurrent modules";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Goals"){
		kv.section="Goals";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Side effects"){
		kv.section="Side effects";kv.subsection="";kv.key="Counter";
	}else
	if(kv.key=="Failure modes"){
		kv.section="Failure modes";kv.subsection="";kv.key="Counter";
	}

	if(kv.section=="Variables" or kv.section=="Parameters" or kv.section=="Concurrent modules" or kv.section=="Failure modes"){
		if(kv.key=="Name"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Resources"){
		if(kv.key=="Resource"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Preconditions"){
		if(kv.key=="Precondition name"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Concurrent conditions"){
		if(kv.key=="Condition name"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Goals"){
		if(kv.key=="Goal name"){
			kv.subsection=kv.value;
		}
	}else
	if(kv.section=="Side effects"){
		if(kv.key=="Parameter"){
			kv.subsection=kv.value;
		}
	}
}

void PlpLoader::parse(){
	vector<string> lines;
	split( lines, _plp_text, boost::is_any_of("\n"));
	KeyValue kv;
	BOOST_FOREACH(string line, lines){
		trim(line);
		plp::parse(kv,line);
		if(kv.key.empty())continue;
		check_if_plp_section(kv);
		_plp_map[kv.full_key()]=kv.value;
	}
}
bool PlpLoader::is_need_header()const{
	if(starts_with(_plp_text, "#! type plp")) return false;
	return true;
}
}


