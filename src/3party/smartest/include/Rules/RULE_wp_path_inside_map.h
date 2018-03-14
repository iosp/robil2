/*
 * RULE_WP_path_inside_map.h
 *
 *  Created on: Jun 23, 2014
 *      Author: userws3
 */

#ifndef RULE_WP_PATH_INSIDE_MAP_H_
#define RULE_WP_PATH_INSIDE_MAP_H_

#include "Rule.h"
class Rule_wp_path_inside_map : public Rule
{
public:
	Rule_wp_path_inside_map();
	virtual ~Rule_wp_path_inside_map();
	bool isRuleValid(SFV *sfv);
};


#endif /* RULE_WP_PATH_INSIDE_MAP_H_ */
