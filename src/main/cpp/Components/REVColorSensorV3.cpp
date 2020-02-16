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
#include "rev/ColorMatch.h"


using namespace rev;
using namespace std;
using namespace frc;
using namespace Components;

REVColorSensorV3::REVColorSensorV3(string _name)
	: InputComponent(_name){
	Color = new ColorSensorV3(frc::I2C::Port::kOnboard);
	Name = _name;
}


rev::ColorMatch m_colorMatcher;

  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

 
  string GetColorMatch() {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
  }

  
string REVColorSensorV3:: GetColorMatch(){

  if (matchedColor == kBlueTarget) {
      colorString = "Blue"; 
    }
     else if (matchedColor == kRedTarget) {
      colorString = "Red";
    } 
    else if (matchedColor == kGreenTarget) {
      colorString = "Green";
    }
     else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
    }
     else {
      colorString = "Unknown";
    }

    return();

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