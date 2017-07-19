/*
 * GC.h
 *
 *  Created on: Jul 19, 2017
 *      Author: assaf
 */

#ifndef FRAMEWORK_PP_SRC_COMPONENT_GC_H_
#define FRAMEWORK_PP_SRC_COMPONENT_GC_H_

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <vector>

#include "GoalCalculator.h"

namespace RobilGC
{
	class GoalCalculator
	{
		nav_msgs::OccupancyGrid				_world_map;
		goal_calculator::Map				_cell_map;
		goal_calculator::GoalCalculator *	_gc;
		vector<goal_calculator::Point_2d>	_path;

		void updateCellMap()
		{
			goal_calculator::Point_2d origin = goal_calculator::Point_2d(_world_map.info.origin.position.x,_world_map.info.origin.position.y);
			tf::Quaternion q;
			q.setW(_world_map.info.origin.orientation.w);
			q.setX(_world_map.info.origin.orientation.x);
			q.setY(_world_map.info.origin.orientation.y);
			q.setZ(_world_map.info.origin.orientation.z);
			double resolution = _world_map.info.resolution, heading = tf::getYaw(q);

			_cell_map = goal_calculator::Map(_world_map.info.width, _world_map.info.height, origin, heading, resolution);

			for(int x = 0; x < _cell_map.w; x++)
			{
				for(int y = 0; y < _cell_map.h; y++)
				{
					size_t index = x + (_cell_map.w * y);
					double coor_x = _cell_map.offset.x + (x * _cell_map.resolution);
					double coor_y = _cell_map.offset.y + (y * _cell_map.resolution);

					// any value more than 50 is  for occupied cell
					if(_world_map.data[index] < 50)
						_cell_map.set_free_value(_cell_map(coor_x,coor_y));
					else
						_cell_map.set_occupied_value(_cell_map(coor_x,coor_y));
				}
			}
		}

		void drawAccessiblePoints(goal_calculator::Point_2d & robot)
		{
			_cell_map.select_accessible_points(robot);
		}

	public:
		GoalCalculator():_gc(new GoalCalculator()),_cell_map(0,0,goal_calculator::Point_2d(0,0),0.0,0.0){}
		virtual ~GoalCalculator()
		{
			delete _gc;
		}

		void updateMap(nav_msgs::OccupancyGrid & new_map, geometry_msgs::PoseWithCovarianceStamped & new_pose)
		{
			goal_calculator::Point_2d pose(new_pose.pose.pose.position.x, new_pose.pose.pose.position.y);
			size_t map_size = new_map.data.size();
			bool map_changed = (map_size != _world_map.data.size()) or (memcmp((void*)new_map.data.data(), (void*)_world_map.data.data(), map_size));

			if(map_changed or (not _cell_map.is_accessible(pose)))
			{
				_world_map = new_map;
				updateCellMap();
				drawAccessiblePoints(pose);
			}
		}

		void updatePath(nav_msgs::Path & gotten_path)
		{
			for(size_t i = 0; i < gotten_path.poses.size(); i++)
			{
				_path.push_back(goal_calculator::Point_2d(gotten_path.poses[i].pose.position.x,
						gotten_path.poses[i].pose.position.y));
			}
		}
	};
}

#endif /* FRAMEWORK_PP_SRC_COMPONENT_GC_H_ */
