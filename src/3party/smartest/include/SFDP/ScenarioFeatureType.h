/*
 * ScenarioFeatureType.h
 *
 *  Created on: Feb 5, 2014
 *      Author: userws1
 */

#ifndef SCENARIOFEATURETYPE_H_
#define SCENARIOFEATURETYPE_H_

#include <boost/enum.hpp>
#include <cstring>

BOOST_ENUM(ScenarioFeatureType,
		(unknown_feature)
		(topographic_map)
		(number_of_objects)

		(object_i_type)
		(object_i_scaling_factor)
		(object_i_location_on_the_X_axis)
		(object_i_location_on_the_Y_axis)
		(object_i_location_on_the_Z_axis)
		(object_i_location_Roll)
		(object_i_location_Pitch)
		(object_i_location_Yaw)

		(number_of_light_sources)
		(light_source_i_type)
		(light_source_i_location_on_the_X_axis)
		(light_source_i_location_on_the_Y_axis)
		(light_source_i_location_on_the_Z_axis)
		(light_source_i_range)
		(light_source_i_direction_on_X_axis)
		(light_source_i_direction_on_Y_axis)
		(light_source_i_light_cone)

		(mass_link_i_mass_deviation)
		(mass_link_i_inertia_deviation_Ixx_component)
		(mass_link_i_inertia_deviation_Iyy_component)
		(mass_link_i_inertia_deviation_Izz_component)
		(mass_link_i_location_deviation_X)
		(mass_link_i_location_deviation_Y)
		(mass_link_i_location_deviation_Z)
		(mass_link_i_location_deviation_Roll)
		(mass_link_i_location_deviation_Pitch)
		(mass_link_i_location_deviation_Yaw)


		(friction_link_friction_deviation)
		(sensor_link_i_location_deviation_X)
		(sensor_link_i_location_deviation_Y)
		(sensor_link_i_location_deviation_Z)
		(sensor_link_i_location_deviation_Roll)
		(sensor_link_i_location_deviation_Pitch)
		(sensor_link_i_location_deviation_Yaw)

		(initial_platform_position_on_map_X_axis)
		(initial_platform_position_on_map_Y_axis)
		(initial_platform_azimut_on_map)

		(number_of_way_points)
		(wp_i_relative_distance)
		(wp_i_relative_angle)
		(wp_i_velocity)
		(clouds)

		(number_of_obstacles_on_path)
		(obstacle_on_path_i_type)
		(obstacle_on_path_i_scaling_factor)
		(obstacle_on_path_i_location_along_the_path)
		(obstacle_on_path_i_location_perpendicular_to_the_path)
		(obstacle_on_path_i_location_on_the_Z_axis)
		(obstacle_on_path_i_location_Roll)
		(obstacle_on_path_i_location_Pitch)
		(obstacle_on_path_i_location_Yaw)
)


#endif /* SCENARIOFEATURETYPE_H_ */
