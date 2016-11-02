/*
 * main.cpp
 *
 *  Created on: May 7, 2014
 *      Author: dan
 */

#include "PLP.h"

int main() {
	cout<<"START"<<endl;
	ifstream file("s1.plp");
	if(file){
		PLPParser parser(file);
		parser.read_all();
		bool res = parser.create_structures();
		if(res){
			std::cout<<"SUCCESS"<<endl;
			std::cout<<parser.plp();cout<<endl;
			PLPCompiler compiler;
			int error_code=0;
			vector<MonitorningScript> ms = compiler.compile(parser.plp(), error_code);
			if(not error_code){
				for(size_t i=0;i<ms.size();i++){
					std::cout<<"-------------\n"<<ms[i]<<endl;
				}
			}else{
				cout<<"Compilation error"<<endl;
			}
		}else{
			cout<<"Parsing Error"<<endl;
		}
	}else{
		cout<<"Cann't open test file"<<endl;
	}
	cout<<"END"<<endl;
	return 0;
}

