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

Author(s): Shruti Venkatramanan, Emily Martinez, Guadalupe Rodriguez
Email: Shruti.venkat05@gmail.com
\********************************************************************/

#include "REVColorSensorV3.h"

using namespace rev;
using namespace std;
using namespace frc;
using namespace Components;

REVColorSensorV3:: InputComponent() {}

REVColorSensorV3::REVColorSensorV3(int _channel, string _name)
	: InputComponent(_name){
    channel = _channel;
	Color = new REVColor;
	Name = _name;
}

double REVColorSensorV3::GetColor(){
	double input = ((double)Color->GetColor()); cvghu 
	return input;
}

double REVColorSensorV3::GetRawColor(){
	double input = ((double)Color->GetRawColor());
	return input;
}

uint32_t REVColorSensorV3::GetProximity(){
    uint32_t input = ((uint32_t)Color->GetProximity());
    return input;

}

string REVColorSensorV3:GetName(){
	return name;
}

REVColorSensorV3::~REVColorSensorV3() {}