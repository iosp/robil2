/*
 * ScenarioFeatureGroupType.h
 *
 *  Created on: Mar 25, 2014
 *      Author: userws1
 */

#ifndef SCENARIOFEATUREGROUPTYPE_H_
#define SCENARIOFEATUREGROUPTYPE_H_

#include <boost/enum.hpp>
#include <SFDP/ScenarioFeatureType.h>
#include <string>

BOOST_ENUM(ScenarioFeatureGroupType,
		(unknown_feature_group)
		(map)
		(lights)
		(objects)
		(object)
		(mass_link_i)
		(friction_link_i)
		(sensor_link_i)
		(platform_pose)
		(obstacle_on_path)
		(obstacles_on_path)
		(Path)
		(WayPoint)
		(SFV)
)



#endif /* SCENARIOFEATUREGROUPTYPE_H_ */
