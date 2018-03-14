/*
 * NumberSampler.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: userws1
 */

#include <utils/NumberSampler.h>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>

NumberSampler::NumberSampler() {
	seed= 0;
}

NumberSampler::~NumberSampler() {
}

void setMinimumMaximum(int &mini,int& maxi,int &param1,int &param2)
{
	if(param1>param2)
	{
		mini=param2;
		maxi=param1;
	}else{
		mini=param1;
		maxi=param2;
	}
}

void setMinimumMaximum(float& mini,float& maxi,float param1,float param2)
{
	if(param1>param2)
	{
		mini=param2;
		maxi=param1;
	}else{
		mini=param1;
		maxi=param2;
	}
}

int NumberSampler::uniformDiscreteDistribution(int distParam1, int distParam2)
{
	if(distParam1==distParam2)
		return(distParam1);

	int mini,maxi;
	setMinimumMaximum(mini,maxi,distParam1,distParam2);
	boost::uniform_int<> dist(mini,maxi);

	gen.seed((++seed) + time(NULL));
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > generator(gen, dist);
	return generator();
}

float NumberSampler::uniformContinuousDistribution(float distParam1, float distParam2)
{
	if(distParam1==distParam2)
		return(distParam1);

	float mini=0,maxi=0;
	setMinimumMaximum(mini,maxi,distParam1,distParam2);
	boost::uniform_real<> dist(mini,maxi);
	gen.seed((++seed) + time(NULL));
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > generator(gen, dist);
	return generator();
}


float NumberSampler::normalContinuousDistribution(float distParam1, float distParam2)
{
	if(distParam2==0)
		return(distParam1);

	boost::normal_distribution<> dist(distParam1,distParam2);
	gen.seed((++seed) + time(NULL));
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > generator(gen, dist);
	return generator();
}
