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

AxisControl::AxisControl(Joystick *_joy, string _name, int _axis, double _deadZone, bool _reversed, double _powerMultiplier, ActiveCollection* ac, bool _useOverdrive)
	: ControlItem(_joy, _name, _reversed, _powerMultiplier, ac)
{
	axis = _axis;
	deadZone = _deadZone;
	isLift = false;
	m_activeCollection = ac;
	useOverdrive = _useOverdrive;
}

double AxisControl::Update(double _dTime)
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
		else if(!m_activeCollection->GetActiveGoal()->GetStatus() == Goal::eActive){
			double currentVal = ((PotentiometerItem*)m_activeCollection->Get("pot"))->Get();
			//TODO: use PID. this is a gross temp fix
			#if 0
			if(!isIdle){
				targetVal = currentVal;
				isIdle = true;
				return currentPow;
				Log::General("!isIdle", true);
				SmartDashboard::PutBoolean("IS WORKING", true);
			}
			double err = (targetVal - currentVal);
			Log::General("error: " + to_string(err));
			currentPow = err * gane;
			Log::General("SETTING CURRENT POW: " + to_string(currentPow));
			#endif
			//SetToComponents(bias);
			return currentPow;
		}
		return currentPow;
	}
	else{
		isIdle = false;
	}
	double dz = deadZone + MINIMUM_JOYSTICK_RETURN;
	double val = ((abs(raw) - dz) * (pow(1-dz, -1)) * getSign(raw)) * powerMultiplier;

	bool overdrive = m_activeCollection->GetOverdrive();
	if(useOverdrive && overdrive && abs(raw) > .95)
	{
		overdriveModifier += .01;
		if(overdriveModifier > 1.0 - powerMultiplier) overdriveModifier = 1.0 - powerMultiplier;
	}
	else
	{
		if(overdriveModifier > 0)
			overdriveModifier -= .01;
		if(overdriveModifier < 0)
			overdriveModifier = 0;
	}
	double signedOverdriveModifier = overdriveModifier * getSign(raw);
	
	
	if(reversed)
	{
		val = -val;
		signedOverdriveModifier = -signedOverdriveModifier;
	}
		
	currentPow = val + signedOverdriveModifier;
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

