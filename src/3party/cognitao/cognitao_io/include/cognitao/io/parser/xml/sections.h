/*
 * tao_parser.h
 *
 *  Created on: Nov 16, 2015
 *      Author: dan
 */

#ifndef SRC_TAO_SECTIONS_PARSER_H_
#define SRC_TAO_SECTIONS_PARSER_H_

#include "core.h"

namespace cognitao{ namespace io{ namespace parser{ namespace xml{ namespace sections{

using namespace core;

void path_parsing( XMLTags& tags, TagPair path, Context& context );
void includes_parsing( XMLTags& tags, TagPair path, Context& context );
void subs_parsing( XMLTags& tags, TagPair subs, Context& context );
void machine_parsing( XMLTags& tags, TagPair machine, Context& context );
void machines_parsing( XMLTags& tags, TagPair machines, Context& context );
enum ParsingMachinePolicy{ PMP_PARSE, PMP_DO_NOT_PARSE, PMP_OPTIONAL};
void tao_parsing( XMLTags& tags, TagPair tao, Context& context, ParsingMachinePolicy process_machines = PMP_PARSE );
void move_machines_to_context( XMLTags& tags, TagPair machine, Context& context, bool search_inside );
void create_reference_tag( const string& id, const TagPair& target, Tag& tag, const Context& context );

}}}}}


#endif /* SRC_TAO_SECTIONS_PARSER_H_ */
