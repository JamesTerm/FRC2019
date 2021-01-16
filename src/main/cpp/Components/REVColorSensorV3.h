/****************************** Header ******************************\
Class Name: REVColorSensorV3 inherits OutputComponent
File Name:	REVColorSensorV3.h
Summary: Abstraction for all programmable robot output components.
Project:     BroncBotzFRC2020
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Emily Martinez, Guadalupe Rodriguez, Shruti Venkatramanan.
Email: shruti.venkat05@gmail.com
\********************************************************************/


#ifndef SRC_COMPONENTS_REVColorSensorV3_H_
#define SRC_COMPONENTS_REVColorSensorV3_H_

#include <rev/ColorSensorV3.h>
#include "rev/ColorMatch.h"
#include <frc/util/color.h>

#include "InputComponent.h"

using namespace std;
using namespace frc;
using namespace rev;
namespace Components{
class REVColorSensorV3: public InputComponent{
private:
    ColorSensorV3 *Color;
	string Name;
	rev::ColorMatch m_colorMatcher;
	frc::Color kYellowTarget;
	frc::Color kRedTarget;
	frc::Color kGreenTarget;
	frc::Color kBlueTarget;


public:
	REVColorSensorV3(string _name);
	frc::Color GetColor();
	virtual double Get() override;
	virtual void DeleteComponent() override;
	string GetColorMatch();
	
	uint32_t GetProximity();
	void ConfigureColorSensor();
	virtual string GetName() override;
	virtual ~REVColorSensorV3();
	ColorSensorV3 *GetREVColorSensorV3() { return Color;}
    void ConfigureProximitySensor();
};
}

#endif /* SRC_COMPONENTS_REVColorSensorV3_H_ */