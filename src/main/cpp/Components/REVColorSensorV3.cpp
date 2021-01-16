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
    //Fix error
	Color = new ColorSensorV3(frc::I2C::Port::kOnboard);
	Name = _name;
    kBlueTarget = frc::Color(0.143, 0.427, 0.429);
    kGreenTarget = frc::Color(0.197, 0.561, 0.240);
    kRedTarget = frc::Color(0.561, 0.232, 0.114);
    kYellowTarget = frc::Color(0.361, 0.524, 0.113);
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
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

double REVColorSensorV3::Get(){

  string colorString = "";
    double Conf = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(GetColor(), Conf);
    
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

  int Color = 0;
  string C = colorString;
  if(C == "Blue")
  {
    Color = 1;
  }
  else if(C == "Red")
  {
    Color = 2;
  }
  else if(C == "Green")
  {
    Color = 3;
  }
  else if(C == "Yellow")
  {
    Color = 4;
  }
  return Color;
}

void REVColorSensorV3::DeleteComponent()
{
  delete Color;
  delete this;
}

REVColorSensorV3::~REVColorSensorV3() {}