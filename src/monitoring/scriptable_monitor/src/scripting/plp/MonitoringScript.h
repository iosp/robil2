/*
 * MonitoringScript.h
 *
 *  Created on: May 7, 2014
 *      Author: dan
 */

#ifndef MONITORINGSCRIPT_H_
#define MONITORINGSCRIPT_H_

#include <sstream>
#include <map>
#include <list>
#include <vector>
#include <algorithm>
#include <boost/foreach.hpp>

using namespace std;

class Poroperties{
public:
	vector<string> attr;
	Poroperties& operator<<(const std::string& str){
		attr.push_back(str);
		return *this;
	}
};
inline
ostream& operator<<(ostream& out, const Poroperties& m){
	for(size_t i=0;i<m.attr.size();i++){
		out<<"#! "<<m.attr[i]<<endl;
	}
	return out;
}

class MonitorningScript{
public:
	Poroperties properties;
	vector<string> lines;
	MonitorningScript& operator<<(const string& s){
		lines.push_back(s);
		return *this;
	}

	MonitorningScript& operator<<(const MonitorningScript& s){
		for(size_t i=0;i<s.properties.attr.size();i++){
			properties.attr.push_back(s.properties.attr[i]);
		}
		for(size_t i=0;i<s.lines.size();i++){
			lines.push_back(s.lines[i]);
		}
		return *this;
	}

	vector<string> assignments()const{
		vector<string> ass;
		static const string comp_char = "<>=!";
		BOOST_FOREACH(string line, lines){
			size_t i = line.find('=');
			if(i==string::npos or i==0 or i==line.size()-1){ continue; }
			if(comp_char.find(line[i-1])!=string::npos or comp_char.find(line[i+1])!=string::npos){ continue; }
			ass.push_back(line);
		}
		return ass;
	}
	vector<string> comparisons()const{
		vector<string> com;
		static const string comp_char = "<>=";
		BOOST_FOREACH(string line, lines){
			bool res = false;
			size_t i = line.find('=');
			if(i==string::npos or i==0 or i==line.size()-1) res=true;
			if(comp_char.find(line[i-1])!=string::npos or comp_char.find(line[i+1])!=string::npos) res=true;
			if(res) com.push_back(line);
		}
		return com;
	}
};
inline
ostream& operator<<(ostream& out, const MonitorningScript& m){
	out<<m.properties<<endl;
	for(size_t i=0;i<m.lines.size();i++){
		out<<m.lines[i]<<endl;
	}
	return out;
}

#endif /* MONITORINGSCRIPT_H_ */
