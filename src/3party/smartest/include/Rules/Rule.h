/*
 * Rule.h
 *
 *  Created on: Jun 2, 2014
 *      Author: userws1
 */

#ifndef RULE_H_
#define RULE_H_

class SFV;
class Rule {
public:
	virtual bool isRuleValid(SFV* sfv)=0;
	inline virtual ~Rule(){};
};

#endif /* RULE_H_ */
