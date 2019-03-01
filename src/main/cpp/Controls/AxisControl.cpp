/****************************** Header ******************************\
Class Name:	AxisControl inherits ControlItem
File Name:	AxisControl.cpp
Summary:	Interface for an axis control.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson
Email:	dylantrwatson@gmail.com
\*********************************************************************/


#include <cmath>
#include <iostream>

#include "AxisControl.h"
#include "..\Util\Constants.h"

using namespace std;
using namespace Util;
using namespace Controls;

AxisControl::AxisControl() { }

AxisControl::AxisControl(Joystick *_joy, string _name, int _axis, double _deadZone, bool _reversed, double _powerMultiplier)
	: ControlItem(_joy, _name, _reversed, _powerMultiplier)
{
	axis = _axis;
	deadZone = _deadZone;
}

double AxisControl::Update()
{
	double raw = (*joy).GetRawAxis(axis);
	if (!(abs(raw) > deadZone))
	{
		if (abs(currentPow) > EPSILON_MIN)
			ValueChanged(new TEventArgs<double, AxisControl*>(0, this));
		currentPow = 0;
		previousPow = currentPow;
		return currentPow;
	}
	double dz = deadZone + MINIMUM_JOYSTICK_RETURN;
	double val = ((abs(raw) - dz) * (pow(1-dz, -1)) * getSign(raw)) * powerMultiplier;
	if(reversed)
		val = -val;
	currentPow = val;
	if(abs(previousPow - currentPow) < EPSILON_MIN)
		return currentPow;
	previousPow = currentPow;
	ValueChanged(new TEventArgs<double, AxisControl*>(currentPow, this));
	return currentPow;
}

int AxisControl::getSign(double val)
{
	if(val < 0)
		return -1;
	else if(val > 0)
		return 1;
	else if(val == 0)
		return 0;
	else{
		Log::Error("Something is very broken in the getSign Method in AxisControl...");
		return 0;
	}
}

AxisControl::~AxisControl() { }

