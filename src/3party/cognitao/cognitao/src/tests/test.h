/*
 * test.h
 *
 *  Created on: Mar 16, 2015
 *      Author: dan
 */

#ifndef TESTS_TEST_H_
#define TESTS_TEST_H_

#include <vector>
#include <boost/function.hpp>
#include <string>

typedef boost::function< int (int, char**) > TestFunction;
typedef std::vector<TestFunction> Tests;
typedef std::vector<std::string> TestNames;

struct TestList{
	static Tests& tests(){ static Tests t; return t; }
	static TestNames& names(){ static TestNames t; return t; }
};

struct TestRegistrator{
	TestRegistrator(std::string name, TestFunction f){ TestList::tests().push_back(f); TestList::names().push_back(name); }
};

#endif /* TESTS_TEST_H_ */
