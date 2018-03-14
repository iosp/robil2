/*
 * ScenarioFeatureDistributionType.h
 *
 *  Created on: Feb 5, 2014
 *      Author: userws1
 */

#ifndef SCENARIOFEATUREDISTRIBUTIONTYPE_H_
#define SCENARIOFEATUREDISTRIBUTIONTYPE_H_

#include <boost/enum.hpp>
#include <string>

BOOST_ENUM(ScenarioFeatureDistributionType,
		(unknown_distribution)
		(uniform_discrete)
		(uniform_continuous)
		(normal_continuous)
)

#endif /* SCENARIOFEATUREDISTRIBUTIONTYPE_H_ */
