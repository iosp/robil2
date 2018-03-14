/*
 * SFVmass_link.h
 *
 *  Created on: Jul 30, 2014
 *      Author: userws3
 */

#ifndef SFVMASS_LINK_H_
#define SFVMASS_LINK_H_

#include <iostream>
#include <string>

#include "SFV/sfvSubGroup.h"
#include "SFDP/ScenarioFeature.h"
#include "SFV/SFV.h"


class SFVmass_link : public sfvSubGroup {
private :

	ScenarioFeature * mass_deviation;
	ScenarioFeature * inertia_deviation_Ixx_component;
	ScenarioFeature * inertia_deviation_Iyy_component;
	ScenarioFeature * inertia_deviation_Izz_component;
	ScenarioFeature * location_deviation_X;
	ScenarioFeature * location_deviation_Y;
	ScenarioFeature * location_deviation_Z;
	ScenarioFeature * location_deviation_Roll;
	ScenarioFeature * location_deviation_Pitch;
	ScenarioFeature * location_deviation_Yaw;

public :
	SFVmass_link(ScenarioFeatureGroup * scenfeaturesGroup, SFV * parent_SFV);
	SFVmass_link(SFVmass_link * template_SFVmass_link);
	SFVmass_link(TiXmlNode * xml_subGroup, SFV * parent_SFV);

	void initFeaturesMap();

	bool roll();
	TiXmlElement * ToXmlElement(int id);

	~SFVmass_link();

	inline ScenarioFeature * get_MassDeviation()
		{ return(mass_deviation); }

	inline ScenarioFeature * get_InertiaDeviationIxxComponent()
		{ return(inertia_deviation_Ixx_component); }

	inline ScenarioFeature * get_InertiaDeviationIyyComponent()
		{ return(inertia_deviation_Iyy_component); }

	inline ScenarioFeature * get_InertiaDeviationIzzComponent()
		{ return(inertia_deviation_Izz_component); }

	inline ScenarioFeature * get_LocationDeviationX()
		{ return(location_deviation_X); }

	inline ScenarioFeature * get_LocationDeviationY()
		{ return(location_deviation_Y); }

	inline ScenarioFeature * get_LocationDeviationZ()
		{ return(location_deviation_Z); }

	inline ScenarioFeature * get_LocationDeviationRoll()
		{ return(location_deviation_Roll); }

	inline ScenarioFeature * get_LocationDeviationPitch()
		{ return(location_deviation_Pitch); }

	inline ScenarioFeature * get_LocationDeviationYaw()
		{ return(location_deviation_Yaw); }
};



#endif /* SFVMASS_LINK_H_ */
