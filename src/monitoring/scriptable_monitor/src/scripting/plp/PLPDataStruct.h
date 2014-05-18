/*
 * PLPDataStruct.h
 *
 *  Created on: May 6, 2014
 *      Author: dan
 */

#ifndef PLPDATASTRUCT_H_
#define PLPDATASTRUCT_H_


#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <list>
#include <map>
using namespace std;

#define set_name(obj,var,value) if(#var=="name" or #var=="parameter"){obj.var=tr(value,' ','_');obj.vars[#var]=obj.var;}
#define set(obj,var,value) set_name(obj,var,value) else{ obj.var=value;obj.vars[#var]=value; }

struct Variable{
	string name;
	string source;
	string type;
	string lower;
	string upper;
	map<string,string> vars;
};
struct Parameter{
	string name;
	string type;
	string source;
	string lower;
	string upper;
	string frequency;
	map<string,string> vars;
};
struct Resource{
	string name;
	string source;
	string type;
	string minimal_initial_value;
	string maximal_consumption;
	string consumption_speed;
	map<string,string> vars;
};
struct Precondition{
	string name;
	string parameter;
	string minimal_value;
	string max_value;
	map<string,string> vars;
};
struct ConcurentCondition{
	string name;
	string parameter;
	string minimal_value;
	string max_value;
	map<string,string> vars;
};
struct ConcurentModule{
	string name;
	string state;
	map<string,string> vars;
};
struct Goal{
	string name;
	string parameter;
	string minimal_value;
	string max_value;
	string success_probability;
	string max_time_to_complete;
	map<string,string> vars;
};
struct SideEffect{
	string parameter;
	string minimal_value;
	string max_value;
	string probability;
	map<string,string> vars;
};
struct FailureMode{
	string name;
	string severity;
	string error_code;
	string failure_probability;
	map<string,string> vars;
};
struct PLP{
	string name;
	string type;
	vector<Variable> variables;
	vector<Parameter> parameters;
	vector<Resource> resources;
	vector<Precondition> preconditions;
	vector<ConcurentCondition> concurent_conditions;
	vector<ConcurentModule> concurent_modules;
	string goal_repeat;
	string repeat_frequency;
	vector<Goal> goals;
	vector<SideEffect> side_effects;
	vector<FailureMode> failure_modes;
	map<string,string> vars;
};

#define PRINT_FIELD(NAME) if(string("source")==#NAME and v.NAME==""){out<<d<<"\t\t\t"<<#NAME<<": UNKNOWN"; d=",\n";}else{out<<d<<"\t\t\t"<<#NAME<<":"<<v.NAME; d=",\n";}
inline
ostream& operator<<(ostream& out, const Variable& v){
	string d="";
	out<<"\t\t{"<<endl;
	PRINT_FIELD(name)
	PRINT_FIELD(type)
	PRINT_FIELD(source)
	PRINT_FIELD(lower)
	PRINT_FIELD(upper)
	return out<<"\n\t\t}";
}
inline
ostream& operator<<(ostream& out, const Parameter& v){
	string d="";
	out<<"\t\t{"<<endl;
	PRINT_FIELD(name)
	PRINT_FIELD(type)
	PRINT_FIELD(source)
	PRINT_FIELD(lower)
	PRINT_FIELD(upper)
	PRINT_FIELD(frequency)
	return out<<"\n\t\t}";
}
inline
ostream& operator<<(ostream& out, const Resource& v){
	string d="";
	out<<"\n\t\t{"<<endl;
	PRINT_FIELD(name)
	PRINT_FIELD(type)
	PRINT_FIELD(source)
	PRINT_FIELD(minimal_initial_value)
	PRINT_FIELD(maximal_consumption)
	PRINT_FIELD(consumption_speed)
	return out<<"\n\t\t}";
}
inline
ostream& operator<<(ostream& out, const Precondition& v){
	string d="";
	out<<"\t\t{"<<endl;
	PRINT_FIELD(name)
	PRINT_FIELD(parameter)
	PRINT_FIELD(minimal_value)
	PRINT_FIELD(max_value)
	return out<<"\n\t\t}";
}
inline
ostream& operator<<(ostream& out, const ConcurentCondition& v){
	string d="";
	out<<"\t\t{"<<endl;
	PRINT_FIELD(name)
	PRINT_FIELD(parameter)
	PRINT_FIELD(minimal_value)
	PRINT_FIELD(max_value)
	return out<<"\n\t\t}";
}
inline
ostream& operator<<(ostream& out, const ConcurentModule& v){
	string d="";
	out<<"\t\t{"<<endl;
	PRINT_FIELD(name)
	PRINT_FIELD(state)
	return out<<"\n\t\t}";
}
inline
ostream& operator<<(ostream& out, const Goal& v){
	string d="";
	out<<"\t\t{"<<endl;
	PRINT_FIELD(name)
	PRINT_FIELD(parameter)
	PRINT_FIELD(minimal_value)
	PRINT_FIELD(max_value)
	PRINT_FIELD(success_probability)
	PRINT_FIELD(max_time_to_complete)
	return out<<"\n\t\t}";
}
inline
ostream& operator<<(ostream& out, const SideEffect& v){
	string d="";
	out<<"\t\t{"<<endl;
	PRINT_FIELD(parameter)
	PRINT_FIELD(minimal_value)
	PRINT_FIELD(max_value)
	PRINT_FIELD(probability)
	return out<<"\n\t\t}";
}
inline
ostream& operator<<(ostream& out, const FailureMode& v){
	string d="";
	out<<"\t\t{"<<endl;
	PRINT_FIELD(name)
	PRINT_FIELD(severity)
	PRINT_FIELD(error_code)
	PRINT_FIELD(failure_probability)
	return out<<"\n\t\t}";
}

inline
ostream& operator<<(ostream& out, const PLP& plp){
#	define FOR(V, ARR) for(size_t i=0;i<plp.ARR.size();i++){ const V = plp.ARR[i];
#	define PRINT_COLLECTION(TYPE, NAME) \
		out<<"\t"<<#NAME<<":["<<endl;\
		FOR( TYPE& item, NAME ){\
			if(i>0)out<<","<<endl; cout<<item;\
		}}\
		out<<endl;\
		out<<"\t"<<"],"<<endl;
#ifdef PRINT_FIELD
#	undef PRINT_FIELD
#endif
#	define PRINT_FIELD(NAME) out<<"\t"<<#NAME<<":"<<plp.NAME<<","<<endl;
	out<<"{"<<endl;
	PRINT_FIELD(name)
	PRINT_FIELD(type)
	PRINT_FIELD(goal_repeat)
	PRINT_FIELD(repeat_frequency)
	PRINT_COLLECTION( Variable, variables )
	PRINT_COLLECTION( Parameter, parameters )
	PRINT_COLLECTION( Resource, resources )
	PRINT_COLLECTION( Precondition, preconditions )
	PRINT_COLLECTION( ConcurentCondition, concurent_conditions )
	PRINT_COLLECTION( ConcurentModule, concurent_modules )
	PRINT_COLLECTION( Goal, goals )
	PRINT_COLLECTION( SideEffect, side_effects)
	PRINT_COLLECTION( FailureMode, failure_modes)
	return out<<"}"<<endl;
#	undef PRINT_COLLECTION
#	undef PRINT_FIELD
#	undef FOR
}



#endif /* PLPDATASTRUCT_H_ */
