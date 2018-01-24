/*
 * RULE_platform_init_pose_with_no_obj_colisions.h
 *
 *  Created on: Jun 23, 2014
 *      Author: userws3
 */

#ifndef RULE_PLATFORM_INIT_POSE_WITH_NO_OBJ_COLISIONS_H_
#define RULE_PLATFORM_INIT_POSE_WITH_NO_OBJ_COLISIONS_H_

#include "Rule.h"
class Rule_platform_init_pose_with_no_obj_colisions : public Rule
{
public:
	Rule_platform_init_pose_with_no_obj_colisions();
	virtual ~Rule_platform_init_pose_with_no_obj_colisions();
	bool isRuleValid(SFV *sfv);
};

#endif /* RULE_PLATFORM_INIT_POSE_WITH_NO_OBJ_COLISIONS_H_ */
