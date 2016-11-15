/*
 * test.cpp
 *
 *  Created on: Mar 16, 2015
 *      Author: dan
 */

#include "test.h"
#include <iostream>

int main(int a, char** aa){
	Tests& tests = TestList::tests();
	TestNames& names = TestList::names();
	for( size_t i=0;i<tests.size();i++ ){
		std::cout<<"TEST #"<<(i+1)<<" : "<<names[i]<<std::endl;
		int res = tests[i](a, aa);
		std::cout<<"    Result is "<<res<<std::endl;
	}
}


