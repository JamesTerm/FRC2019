/****************************** Header ******************************\
Class Name: REVColorSensorV3 inherits InputComponent
File Name:	REVColorSensorV3.cpp
Summary: Abstraction for the WPIlib Encoder that extends to include
some helper and control methods.
Project:     BroncBotzFRC2020
Copyright (c) BroncBotz.
All rights reserved.
    ----------
   |///   /// |
   |    |     |
   | ________ |
    -----------

Author(s): Shruti Venkatramanan, Emily Martinez, Guadalupe Rodriguez.
Email: Shruti.venkat05@gmail.com
\********************************************************************/

#include "REVColorSensorV3.h"

using namespace rev;
using namespace std;
using namespace frc;
using namespace Components;

REVColorSensorV3::REVColorSensorV3(string _name)
	: InputComponent(_name){
	Color = new ColorSensorV3(frc::I2C::Port::kOnboard);
	Name = _name;
}

frc::Color REVColorSensorV3::GetColor(){
	frc::Color input = (Color->GetColor()); 
	return input;
}


uint32_t REVColorSensorV3::GetProximity(){
    uint32_t input = ((uint32_t)Color->GetProximity());
    return input;

}

string REVColorSensorV3::GetName(){
	return name;
}

REVColorSensorV3::~REVColorSensorV3() {}