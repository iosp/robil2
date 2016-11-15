/*
 * args.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef SRC_TAO_PARSER_ARGS_H_
#define SRC_TAO_PARSER_ARGS_H_

#include "core.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace args{

using namespace core;

void arg_parsing( XMLTags& tags,  TagPair arg_tag, Context& context, bool skip_id = false );
void args_parsing( XMLTags& tags, TagPair main_tag, Context& context );


}}}}}


#endif /* SRC_TAO_PARSER_ARGS_H_ */
