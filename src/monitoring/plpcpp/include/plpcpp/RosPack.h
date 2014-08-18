/*
 * RosPack.h
 *
 *  Created on: Jul 17, 2014
 *      Author: dan
 */

#ifndef ROSPACK_H_
#define ROSPACK_H_

#include "ShellExecutor.h"
#include <sstream>
using namespace std;
class RosPack{
public:
	struct Exception{virtual ~Exception(){};virtual std::string what()const=0;};
#	define DEF_EXC(NAME, DESC) struct Exc_##NAME:public Exception{ std::string what()const{ return DESC;} };
#	define DEF_EXC_FROM(TYPE,NAME) struct Exc_##NAME:public TYPE{ std::string w; Exc_##NAME(std::string w):w(w){} std::string what()const{ return w;} };
		DEF_EXC(RosPackNotFound,"sh: 1: rospack: not found")
		DEF_EXC(PackageNotFound,"[rospack] Error: stack/package ptp not found")
		DEF_EXC(SomethingWrong,"Something wrong")
		DEF_EXC_FROM(Exception, FindProblem)
#	undef DEF_EXC
#	undef DEF_EXC_FROM

	std::string find(std::string name){
		const string error_1 = "sh: 1: rospack: not found\n";
		const string error_2 = "[rospack] Error: stack/package ptp not found\n";
		string res = SHELL_EXEC( "rospack find "+name );
		if(res.length()==0) throw Exc_SomethingWrong();
		if(res==error_1) throw Exc_RosPackNotFound();
		if(res==error_2) throw Exc_PackageNotFound();
		return res;
	}

	static size_t search_last(const std::string& s, char c){
		for(size_t i=s.size()-1;i>=0;i--){
			if(s[i]==c) return i;
			if(i==0) return std::string::npos;
		}
		return std::string::npos;
	}
	static std::string get_first_line(const std::string& s){
		size_t e = s.find('\n');
		if(e==string::npos) return s;
		return s.substr(0,e);
	}
	static std::string get_first_line(const std::string& _s, std::string& rest){
		std::string s = _s;
		size_t e = s.find('\n');
		if(e==string::npos){ rest=""; return s; }
		rest = s.substr(e+1);
		return s.substr(0,e);
	}
	struct FindResult{
		std::string res;
		FindResult(const std::string& r):res(r){}
		operator std::string()const{ return res; }
		FindResult operator[](int i)const{
			std::string line; std::string rest(res);
			while(i>=0){
				line = get_first_line(rest, rest);
				i--;
			}
			return line;
		}
		size_t size()const{
			if(res.size()==0) return 0;
			size_t c=0;
			for(size_t i=0;i<res.size();i++) if(res[i]=='\n') c++;
			return c+1;
		}
	};
	FindResult find_file(std::string name, std::string grep="", bool justText=true){
		const std::string pref = "${ROS:";
		const string fproblem = "find:";
		if(name.substr(0,pref.size())==pref){
			size_t c = name.find("}");
			if(c==std::string::npos) return FindResult("");
			size_t s = pref.size();
			std::string pname = name.substr(s, c-s);
			pname = get_first_line( find(pname) );
			std::string file = name.substr(c+1);
			size_t fstr = search_last(file,'/');
			std::string folder="",fname=file;
			if(fstr!=string::npos){ folder = file.substr(0,fstr); fname=file.substr(fstr+1); }
			std::string command = "find '"+pname+folder+"' -iname '"+fname+"'"; //cout<<"@ "<<command<<endl;
			if(grep.size()>0){ command+=" -print | grep "+grep+""; }
			std::string res = SHELL_EXEC(command);
			if(res.substr(0,fproblem.size())==fproblem) throw Exc_FindProblem(res);
			if(res.size()>=name.size()){ return res; }
		}else{
			size_t fstr = search_last(name,'/');
			std::string folder="",fname=name;
			if(fstr!=string::npos){ folder = name.substr(0,fstr); fname=name.substr(fstr+1); }
			std::string command = "find '"+folder+"' -iname '"+fname+"'"; //cout<<"@ "<<command<<endl;
			if(grep.size()>0){ command+=" -print | grep "+grep+""; }
			std::string res = SHELL_EXEC(command);
			if(res.substr(0,fproblem.size())==fproblem) throw Exc_FindProblem(res);
			if(res.size()>=name.size()){ return res; }
		}
		return FindResult("");
	}
};


#endif /* ROSPACK_H_ */
