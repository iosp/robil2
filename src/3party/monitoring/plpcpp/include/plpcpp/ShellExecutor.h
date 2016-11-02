/*
 * ShellExecutor.h
 *
 *  Created on: Jul 17, 2014
 *      Author: dan
 */

#ifndef SHELLEXECUTOR_H_
#define SHELLEXECUTOR_H_

#include <stdio.h>
#include <sstream>
#include <iostream>

class Shell{
public:
	class Exception{public:virtual ~Exception(){} virtual std::string what()const=0;};
	class Exc_PipeProblems:public Exception{public:std::string what()const{return "Problems with pipe";}};

	std::string run ( std::string command )
	{
	   FILE *fpipe;
	   char line[256];
	   std::stringstream results;
	   //command = command;

	   if ( !(fpipe = (FILE*)popen(command.c_str(),"r")) )
	   {  // If fpipe is NULL
		  perror("Problems with pipe");
		  throw Exc_PipeProblems();
	   }

	   while ( fgets( line, sizeof line, fpipe))
	   {
		 results<<line;
	   }
	   pclose(fpipe);
	   return results.str();
	}
	std::stringstream command;
	std::string* output;
	Shell():output(0){};

	Shell& add(const std::string& cmd){
		if(command.str().length()!=0)command<<"|";
		command<<cmd<<" 2>&1";
		return *this;
	}
	Shell& prev(){
		if(output){ this->add(*output); output=0; }
		return *this;
	}
	Shell& operator|(const char* cmd){
		return prev().add(std::string(cmd));
	}
	Shell& operator|(const std::string& cmd){
		return prev().add(cmd);
	}
	Shell& operator|(std::string& cmd){
		prev();
		output = &cmd;
		return *this;
	}

	Shell& operator>(std::string& cmd){
		prev();
		output = &cmd;
		std::string res = run(command.str());
		if(output) *output = res;
		output=0; command.str("");
		return *this;
	}
	Shell& operator>>(std::string& cmd){
		prev();
		output = &cmd;
		std::string res = run(command.str());
		if(output) *output += res;
		output=0; command.str("");
		return *this;
	}

	virtual ~Shell(){
		if(command.str().length()>0){
			std::string res = run(command.str());
			if(output) *output = res;
		}
	}
	operator std::string(){ std::string var; (*this)>var; return var; }
};


#define SHELL_EXEC( X ) (Shell() | X)


#endif /* SHELLEXECUTOR_H_ */
