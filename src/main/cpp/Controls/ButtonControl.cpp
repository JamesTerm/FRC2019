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

ButtonControl::ButtonControl(Joystick *_joy, string _name, int _button, bool _actOnRelease, bool _reversed, double _powerMultiplier, bool _isSolenoid, ActiveCollection* ac, bool _isOverdrive)
	: ControlItem(_joy, _name, _reversed, _powerMultiplier, ac) {
	button = _button;
	actOnRelease = _actOnRelease;
	isSolenoid = _isSolenoid;
	previous = 0;
	inc = 0.12;
	isAmpRegulated = false;
	powerPort = 20;
	pdp = new PowerDistributionPanel();

	isOverdrive = _isOverdrive;
}

void ButtonControl::DeleteComponent()
{
	delete pdp;
	delete this;
}

double ButtonControl::Update(double _dTime){
	double val = joy->GetRawButton(button); //*< Value recieved from the button
	double tmp = val; //*< Temporary value to consistantly hold the original value of the button -> not affected by reversing the ButtonControl

	if(isOverdrive)
		{
			bool over = !(val < EPSILON_MIN);
			m_activeCollection->SetOverdrive(over);
		}

	/*
	* This is not a ramped control -> it does not speed up while the button is held down like a jet engine
	* AND
	* It is not regulated by ampereage
	* OR
	* It is regulated by ampereage but has not exceeded the ampereage limit
	*/
	if(!isRamp && ((!isAmpRegulated) || !(pdp->GetCurrent(powerPort) > ampLimit))){
		/*
		* If the control is designated reversed, reverse the recieved value of the button
		*/
		if (reversed)
			val = !val; //*< Set the value of the button to the inverse of the recieved value
		current = val * powerMultiplier; //*< This is the power that will be set to the motor based upon the powerMultiplier

		

		/*
		* If the value has not changed since the last calling of the Update function, end the function
		*/
		if (abs(previous - current) < EPSILON_MIN) //If the state of the button has not changed, don't change a thing
			return current;
		if (!actOnRelease && !tmp) //If this is not actOnRelease AND if the button is released CONTINUE to set the previous value from when it was pushed, NOT the current value
			ValueChanged(new TEventArgs<double, ButtonControl*>(previous, this));
		else //This would happen if it has changed and it is either (actOnRelease and released [which would set 0]) OR (!actonRelease and pressed [which would set 1*powerMultiplier])
			ValueChanged(new TEventArgs<double, ButtonControl*>(current, this));
		previous = current;
		return current;
	}
	/*
	* This is a ramped control -> it will speed up while the button is held down like a jet engine
	* AND
	* It is not regulated by ampereage
	* OR
	* It is regulated by ampereage but has not exceeded the ampereage limit
	*/
	else if(isRamp && ((!isAmpRegulated) || !(pdp->GetCurrent(powerPort) > ampLimit))){
		/*
		* If the button is being pressed
		*/
		if(val){

			/*
			* If the difference between the previous value set to the motor and the powermultiplier is greater than the set increment
			*/
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
			/*
			* If the difference between the previous value set to the motor and the powermultiplier is not greater than the set increment
			*/
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
			current = 0.0;
			if(abs(previous - current) > EPSILON_MIN)
					ValueChanged(new TEventArgs<double, ButtonControl*>(current, this));
			previous = 0;
			current = 0;
			return current;
		}
	}
	/*
	* This control is ampereage regulated
	* AND
	* The apereage is exceeding the set limit for the component 
	*/
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
			Log::Error("Error setting default value to binding for " + name + " control!\nYOU MAY HAVE SET A SOLENOID THING TO A MOTOR THING ON BUTTON!");
		}
}

void ButtonControl::SetToSolenoids(DoubleSolenoid::Value value){
	try{
		for(int i=0; i<(int)components.size();i++)
			(*components[i]).Set(value);
	}
	catch(...){
		Log::Error("Error setting value to binding for " + name + " control!\nYOU MAY HAVE SET A SOLENOID THING TO A MOTOR THING ON BUTTON!");
	}
}

void ButtonControl::SetRamp(double _inc){
	if(isSolenoid){
		Log::Error("WHY DID YOU SET A RAMP TO A SOLENOID");
		return;
	}
	isRamp = true;
	inc = _inc;
}

void ButtonControl::SetAmpRegulation(int _powerPort, double _ampLimit){
	if(isSolenoid){
		Log::Error("WHY DID YOU SET AMP REGULATION TO A SOLENOID");
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