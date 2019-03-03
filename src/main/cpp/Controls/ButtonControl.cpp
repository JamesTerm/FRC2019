/****************************** Header ******************************\
Class Name:	ButtonControl inherits ControlItem
File Name:	ButtonControl.cpp
Summary:	Interface for a single button control.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson
Email:	dylantrwatson@gmail.com
\*********************************************************************/

#include <iostream>

#include "ButtonControl.h"

using namespace Controls;

ButtonControl::ButtonControl() {}

ButtonControl::ButtonControl(Joystick *_joy, string _name, int _button, bool _actOnRelease, bool _reversed, double _powerMultiplier, bool _isSolenoid)
	: ControlItem(_joy, _name, _reversed, _powerMultiplier) {
	button = _button;
	actOnRelease = _actOnRelease;
	isSolenoid = _isSolenoid;
	previous = 0;
	inc = 0.12;
	isAmpRegulated = false;
	powerPort = 20;
	pdp = new PowerDistributionPanel();
}

double ButtonControl::Update(){
	/*================================== FROM THE LEGACY CODE ==============================*/
	double val = joy->GetRawButton(button);
	double tmp = val;
	if(!isRamp && ((!isAmpRegulated) || !(pdp->GetCurrent(powerPort) > ampLimit))){
		if (reversed)
			val = !val;
		current = val * powerMultiplier;
		if (abs(previous - current) < EPSILON_MIN) 
			return current;
		if (!actOnRelease && !tmp)
			ValueChanged(new TEventArgs<double, ButtonControl*>(previous, this));
		else
			ValueChanged(new TEventArgs<double, ButtonControl*>(current, this));
		previous = current;
		return current;
	}
	else if(isRamp && ((!isAmpRegulated) || !(pdp->GetCurrent(powerPort) > ampLimit))){
		if(val){
			if(abs(abs(previous) - powerMultiplier) >= inc){
				if(getSign(powerMultiplier) == -1){
					current -= inc;
					ValueChanged(new TEventArgs<double, ButtonControl*>(current, this));
				}
				else if(getSign(powerMultiplier) == 1){
					current += inc;
					ValueChanged(new TEventArgs<double, ButtonControl*>(current, this));
				}
				else{
					if(abs(previous - current) > EPSILON_MIN)
						ValueChanged(new TEventArgs<double, ButtonControl*>(0, this));
				}
				SetToComponents(current);
			}
			else if(!(abs(abs(previous) - powerMultiplier) >= inc)){
				if(abs(previous - current) > EPSILON_MIN){
					current = powerMultiplier;
					ValueChanged(new TEventArgs<double, ButtonControl*>(current, this));
				}
			}
			previous = current;
			return current;
		}
		else if(actOnRelease && !val){
			if(abs(previous - current) > EPSILON_MIN)
					ValueChanged(new TEventArgs<double, ButtonControl*>(current, this));
					//should we add a goal
						//if so, add it
			previous = 0;
			current = 0;
			return current;
		}
	}
	else if(isAmpRegulated && (pdp->GetCurrent(powerPort) > ampLimit)){
		if(val){
			double absPWR = abs(previous) - inc;
			if(getSign(powerMultiplier) == -1)
				absPWR *= -1;
			current = absPWR;
			ValueChanged(new TEventArgs<double, ButtonControl*>(current, this));
			previous = current;
			return current;
		}
		else if (actOnRelease && !val){
			if(abs(previous - current) > EPSILON_MIN)
					ValueChanged(new TEventArgs<double, ButtonControl*>(current, this));
			previous = 0;
			current = 0;
			return current;
		}
	}
	return current;
}

void ButtonControl::SetSolenoidDefault(){
	try
	{
		for(int i=0; i<(int)components.size();i++)
			(*components[i]).DefaultSet();
		}
		catch(...){
			cout << "Error setting default value to binding for " << name << " control!\nYOU MAY HAVE SET A SOLENOID THING TO A MOTOR THING ON BUTTON!" << endl;
		}
}

void ButtonControl::SetToSolenoids(DoubleSolenoid::Value value){
	try{
		for(int i=0; i<(int)components.size();i++)
			(*components[i]).Set(value);
	}
	catch(...){
		cout << "Error setting value to binding for " << name << " control!\nYOU MAY HAVE SET A SOLENOID THING TO A MOTOR THING ON BUTTON!" << endl;
	}
}

void ButtonControl::SetRamp(double _inc){
	if(isSolenoid){
		cout << "WHY DID YOU SET A RAMP TO A SOLENOID" << endl;
		cerr << "WHY DID YOU SET A RAMP TO A SOLENOID" << endl;
		return;
	}
	isRamp = true;
	inc = _inc;
}

void ButtonControl::SetAmpRegulation(int _powerPort, double _ampLimit){
	if(isSolenoid){
		cout << "WHY DID YOU SET AMP REGULATION TO A SOLENOID" << endl;
		cerr << "WHY DID YOU SET AMP REGULATION TO A SOLENOID" << endl;
		return;
	}
	isAmpRegulated = true;
	powerPort = _powerPort;
	ampLimit = _ampLimit;
}

int ButtonControl::getSign(double val){
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

ButtonControl::~ButtonControl() {}