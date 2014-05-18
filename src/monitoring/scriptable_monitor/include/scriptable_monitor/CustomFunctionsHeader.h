
#include <scriptable_monitor/InternalFunction.h>

#define FUNCTION(PlpFunction)\
	class PlpFunction : public InternalFunction {\
	public:\
		virtual string functionName();\
	protected:\
		virtual void process(Parameters& input, Parameters& output);\
	};

