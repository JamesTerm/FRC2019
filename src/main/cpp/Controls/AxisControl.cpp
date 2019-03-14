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
using namespace Components;

AxisControl::AxisControl() { }

AxisControl::AxisControl(Joystick *_joy, string _name, int _axis, double _deadZone, bool _reversed, double _powerMultiplier)
	: ControlItem(_joy, _name, _reversed, _powerMultiplier)
{
	axis = _axis;
	deadZone = _deadZone;
	isLift = false;
}

double AxisControl::Update()
{
	double raw = (*joy).GetRawAxis(axis);
	if (!(abs(raw) > deadZone))
	{
		if(!isLift){
			if (abs(currentPow) > EPSILON_MIN)
				ValueChanged(new TEventArgs<double, AxisControl*>(0, this));
			currentPow = 0;
			previousPow = currentPow;
			return currentPow;
		}
		else{
			double currentVal = ((PotentiometerItem*)m_activeCollection->Get("pot"))->Get();
			if(!isIdle){
				targetVal = currentVal;
				isIdle = true;
				return currentPow;
			}
			double err = (targetVal - currentVal)/targetVal;
			currentPow = err * gane;
			SetToComponents(currentPow);
			return currentPow;
		}
	}
	else{
		isIdle = false;
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

void AxisControl::SetLift(double _gane, ActiveCollection* activeCollection){
	gane = _gane;
	isLift = true;
	m_activeCollection = activeCollection;
}

AxisControl::~AxisControl() { }

