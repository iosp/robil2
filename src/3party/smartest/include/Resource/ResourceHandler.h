/*
 * ResourceHandler.h
 *
 *  Created on: Feb 24, 2014
 *      Author: userws1
 */

#ifndef RESOURCEHANDLER_H_
#define RESOURCEHANDLER_H_
#include <string>
#include <map>
#include <vector>
#include <iostream>

class ResourceHandler {

private:
    // Private Constructor
	ResourceHandler(std::string resources_file_path);
	virtual ~ResourceHandler();

	std::string my_RobotModelsFolderURL;
	std::string my_RobotName;
	std::string my_RobotPlatformName;
	std::vector<std::string *> * my_RobotSensorsNames;

	std::string my_WorldModelsFolderURL;
	std::map <std::string, std::map<int,std::string>*>* m_resourceMap;

	std::string my_Robot_PyInterface;

	std::string getResource(std::string name,int id);

public:
    static ResourceHandler& getInstance(std::string resources_file_path)
    {
        static ResourceHandler instance(resources_file_path);
        return instance;
    }

    std::string getTerrainById(int id);
    std::string getObjectById(int id);
    std::string getLightById(int id);


	inline std::string getWorldModelsFolderURL()
				{	if ( my_WorldModelsFolderURL != "")  { return my_WorldModelsFolderURL; }
				else  { std::cout << "\033[1;31m No WorldModelsFolderURL was set \033[1;31m"<< std::endl; 	return 0;   } }


	inline std::string getRobotModelsFolderURL()
				{	if ( my_RobotModelsFolderURL != "")  { return my_RobotModelsFolderURL; }
					else  { std::cout << "\033[1;31m No RobotModelsFolderURL was set \033[1;31m"<< std::endl; 	return 0;   } }


	inline std::string getRobotName()
				{	if ( my_RobotName != "")  { return my_RobotName; }
					else  { std::cout << "\033[1;31m No RobotName was set \033[1;31m"<< std::endl; 	return 0;   } }


	inline std::string getRobotPlatformName()
				{	if ( my_RobotPlatformName != "")  { return my_RobotPlatformName; }
					else  { std::cout << "\033[1;31m No RobotPlatformName was set \033[1;31m"<< std::endl; 	return 0;   } }


	inline std::vector <std::string *> * getRobotSensorsNames()
				{	if ( ! my_RobotSensorsNames->empty() )  { return my_RobotSensorsNames; }
					else  { std::cout << "\033[1;31m No RobotSensorsNames where set \033[1;31m"<< std::endl; 	return 0;   } }

	inline std::string getRobotPyInterface()
				{	if ( my_Robot_PyInterface != "")  { return my_Robot_PyInterface; }
					else  { std::cout << "\033[1;31m No my_Robot_PyInterface was set \033[1;31m"<< std::endl; 	return 0;   } }


};

#endif /* RESOURCEHANDLER_H_ */



