/*
 * core.h
 *
 *  Created on: Nov 24, 2015
 *      Author: dan
 */

#ifndef INCLUDE_COGNITAO_CORE_H_
#define INCLUDE_COGNITAO_CORE_H_


#include <iostream>
#include <sstream>

#include <map>
#include <set>
#include <deque>
#include <list>
#include <vector>
#include <algorithm>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/enable_shared_from_this.hpp>

struct NOT_IMPLEMENTED{};

//--------- trick for define foreach statement -------------
//--- source: https://svn.boost.org/trac/boost/timeline?from=2012-10-11T11%3A53%3A54-04%3A00&precision=second
#include <boost/foreach.hpp>
#include <boost/version.hpp>
namespace boost {
	#if BOOST_VERSION != 104900
		namespace BOOST_FOREACH = foreach;
	#endif
}
#define foreach BOOST_FOREACH
//----------------------------------------------------------

#define foreachitem BOOST_FOREACH
#define foreachindex(INDEX, ARR) for(size_t INDEX=0;INDEX<(ARR).size();INDEX++)

#define THIS shared_from_this()
#define CONST_SHARED(X) public boost::enable_shared_from_this<const X>
#define SHARED(X) public boost::enable_shared_from_this<X>

using namespace std;
using namespace boost;
using namespace posix_time;

#define MEMORY_LEEKS_CHECK 0



#endif /* INCLUDE_COGNITAO_CORE_H_ */
