/*
 * NumberSampler.h
 *
 *  Created on: Feb 5, 2014
 *      Author: userws1
 */

#ifndef NUMBERSAMPLER_H_
#define NUMBERSAMPLER_H_
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/random/mersenne_twister.hpp>

/**
 * A Number sampler class. This class generates random numbers according to a given distribution
 */

class NumberSampler {

	/**
	* random number generator.
	* boost mt19937 number generator.
	* see http://www.boost.org/doc/libs/1_46_1/doc/html/boost_random/tutorial.html for a more details.
	*/
	boost::mt19937 gen;

	/**
	* random seed number
	* a seed number is provides the random number generator with a fresh base to start calculations from.
	* the seed is updated after each roll in order to minimize the possibility of sampling of the distribution from the same spot.
	*/
	unsigned int seed;

	 /**
	* A constructor.
	*/
	NumberSampler();

	 /**
	* A destructor.
	*/
	virtual ~NumberSampler();
    // Stop the compiler generating methods of copy the object
	NumberSampler(NumberSampler const& copy);            // Not Implemented
	NumberSampler& operator=(NumberSampler const& copy); // Not Implemented

public:
    static NumberSampler& getInstance()
    {
        static NumberSampler instance;
        return instance;
    }


	 /**
	* a number sampling method that takes the distribution bounds and samples an integer in a uniform fashion.
	* @param distParam1 an integer argument, a lower bound for the distribution
	* @param distParam2 an integer argument, an upper bound for the distribution
	* @see uniformRealDistribution()
	* @return an integer sampled uniformly between bounds
	*/
	int uniformDiscreteDistribution(int distParam1, int distParam2);

	 /**
	* a number sampling method that takes the distribution bounds and samples a real number in a uniform fashion.
	* @param distParam1 a real number argument, a lower bound for the distribution
	* @param distParam2 a real number argument, an upper bound for the distribution
	* @see uniformIntDistribution()
	* @return a a real number sampled uniformly between bounds
	*/
	float uniformContinuousDistribution(float distParam1, float distParam2);


	 /**
	* a number sampling method that takes the distribution bounds and samples a real number in a normal fashion.
	* @param distParam1 a real number argument, a lower bound for the distribution
	* @param distParam2 a real number argument, an upper bound for the distribution
	* @see uniformIntDistribution()
	* @return a a real number sampled uniformly between bounds
	*/
	float normalContinuousDistribution(float distParam1, float distParam2);

};

#endif /* NUMBERSAMPLER_H_ */
