/*
 * arguments_substitution.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef SRC_ARGUMENTS_SUBSTITUTION_H_
#define SRC_ARGUMENTS_SUBSTITUTION_H_

#include "core.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace arguments{

using namespace core;

string substitut_argument( const string& text, const Context& context, Requerments req );
void substitut_arguments( string& text , const Context& context, Requerments req );
void substitut_arguments( XMLTags& tags, TagPtr start, TagPtr end, const Context& context, Requerments req );

void collect_blocks( XMLTags& tags, TagPair tag, Context& context );
void replace_blocks( XMLTags& tags, TagPair tag, Context& context );
void remove_blocks_envelope( XMLTags& tags, TagPair tag, Context& context );
void expand_blocks( XMLTags& tags, TagPair tag, const Context& global_context );

void apply_if_conditions( XMLTags& tags, TagPair tag, const Context& context );

}}}}}

#endif /* SRC_ARGUMENTS_SUBSTITUTION_H_ */
