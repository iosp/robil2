/*
 * subs.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef SRC_TAO_PARSER_SUBS_H_
#define SRC_TAO_PARSER_SUBS_H_

#include "core.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace subs{
using namespace core;


void sub_parsing( XMLTags& tags, TagPair sub, const Context& context );
void sub_parsing( const std::string sub_id, const Context& context, XMLTags& tags);
void expand_subs( XMLTags& tags, TagPair& main_tag, const Context& context);

}}}}}


#endif /* SRC_TAO_PARSER_SUBS_H_ */
