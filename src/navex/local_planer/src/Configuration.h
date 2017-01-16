/*
 * Configuration.h
 *
 *  Created on: Jan 12, 2017
 *      Author: dan
 */

#ifndef LOCAL_PLANER_SRC_CONFIGURATION_H_
#define LOCAL_PLANER_SRC_CONFIGURATION_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <local_planer/local_planerConfig.h>

class Configuration
{
public:
	typedef local_planer::local_planerConfig Config;

private:
	Config* conf;
	void callback(Config &config, uint32_t level)
	{
		*conf = config;
	}

public:
	Configuration(Config& conf)
	{
		init(&conf);
	}

private:
	void init(Config* conf)
	{
		this->conf = conf;
		dynamic_reconfigure::Server<Config> srv;
		dynamic_reconfigure::Server<Config>::CallbackType f;
		f = boost::bind(&Configuration::callback, this, _1, _2);
		srv.setCallback(f);
	}
};


#endif /* LOCAL_PLANER_SRC_CONFIGURATION_H_ */
