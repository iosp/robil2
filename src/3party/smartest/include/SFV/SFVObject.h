/*
 * SFVObject.h
 *
 *  Created on: Jul 31, 2014
 *      Author: userws3
 */

#ifndef SFVOBJECT_H_
#define SFVOBJECT_H_


#include "SFV/sfvSubGroup.h"
#include "SFDP/ScenarioFeature.h"
#include "SFV/SFV.h"


class SFVObject : public sfvSubGroup  {
private :
	ScenarioFeature * my_object_type;
	ScenarioFeature * my_scaling_factor;
	ScenarioFeature * my_location_on_the_X_axis;
	ScenarioFeature * my_location_on_the_Y_axis;
	ScenarioFeature * my_location_on_the_Z_axis;
	ScenarioFeature * my_location_Roll;
	ScenarioFeature * my_location_Pitch;
	ScenarioFeature * my_location_Yaw;

	std::map<char,float> * Implicit_Object_xyz;

public :
	SFVObject(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV);
	SFVObject(SFVObject * template_SFVObject);
	SFVObject(TiXmlNode * xml_subGroup, SFV * parent_SFV);

	void initFeaturesMap();

	bool roll();
	std::map<char,float> * get_Object_xyz();
	TiXmlElement * ToXmlElement(int id);

	~SFVObject();


	inline ScenarioFeature * get_ObjectType()
		{ return(my_object_type); }

	inline ScenarioFeature * get_ScalingFactor()
		{ return(my_scaling_factor); }

	inline ScenarioFeature * get_LocationOnTheXaxis()
		{ return(my_location_on_the_X_axis); }

	inline ScenarioFeature * get_LocationOnTheYaxis()
		{ return(my_location_on_the_Y_axis); }

	inline ScenarioFeature * get_LocationOnTheZaxis()
		{ return(my_location_on_the_Z_axis); }

	inline ScenarioFeature * get_LocationRoll()
		{ return(my_location_Roll); }

	inline ScenarioFeature * get_LocationPitch()
		{ return(my_location_Pitch); }

	inline ScenarioFeature * get_LocationYaw()
		{ return(my_location_Yaw); }
};

#endif /* SFVOBJECT_H_ */
