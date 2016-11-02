#define FUNCTION(PlpFunction)\
	REGISTER_INTERNAL_FUNCTION(PlpFunction)\
	string PlpFunction::functionName()\
	{\
		return #PlpFunction;\
	}\
	void PlpFunction::process(Parameters& input, Parameters& output)

