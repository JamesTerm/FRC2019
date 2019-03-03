/****************************** Header ******************************\
Class Name:	ToggleButtonControl inherits ControlItem
File Name:	ToggleButtonControl.cpp
Summary:	Interface for a toggle button control.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson, Ian Poll
Email:	dylantrwatson@gmail.com, irobot983@gmail.com
\*********************************************************************/

#include "ToggleButtonControl.h"

using namespace std;
using namespace Controls;

ToggleButtonControl::ToggleButtonControl() { }

ToggleButtonControl::ToggleButtonControl(Joystick *_joy, string _name, int _button, bool _IsReversed, double _powerMultiplier): ControlItem(_joy, _name, _IsReversed, _powerMultiplier)
{
	button = _button;
}

double ToggleButtonControl::Update()
{
	bool val = joy->GetRawButton(button);
	if (val == previousState) return val * powerMultiplier;
	if (val != previousState && val == true)
	{
		if (toggleOn)
		{
			toggleOn = false;
			ValueChanged(new TEventArgs<double, ToggleButtonControl*>(0, this));
		}
		else
		{
			toggleOn = true;
			ValueChanged(new TEventArgs<double, ToggleButtonControl*>(val * powerMultiplier, this));
		}
	}
	previousState = val;
	return current;
}

ToggleButtonControl::~ToggleButtonControl() { }

