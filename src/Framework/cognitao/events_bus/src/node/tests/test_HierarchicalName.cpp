/*
 * test_HierarchicalName.cpp
 *
 *  Created on: Oct 27, 2015
 *      Author: dan
 */

#include <events_bus/HierarchicalName.h>
#include <iostream>
#include <boost/foreach.hpp>
using namespace cognitao;
using namespace std;




int main(int a, char** aa)
{

//	cout<< HierarchicalName ("test/a/b/c/").name() << endl;
//	cout<< HierarchicalName ("test/a/b/c/").full_name() << endl;
//	cout<< HierarchicalName ("test/a/b/c/").parent() << endl;

	vector<string> s = HierarchicalName ("/a/b/c/d").split();
	BOOST_FOREACH( string st, s )
	{
		cout<<" -- "<<st<<endl;
	}

	cout<< ": " << HierarchicalName ("/").parent() << endl;

	return 0;
}





