/****************************** Header ******************************\
Class Name: DoubleSolenoidItem inherits OutputComponent
File Name:	DoubleSolenoidItem.cpp
Summary: Abstraction for the WPIlib DoubleSolenoid that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#include <cmath>
#include <iostream>

#include "DoubleSolenoidItem.h"
#include "../Util/Constants.h"

using namespace std;
using namespace frc;
using namespace Components;
using namespace Util;

DoubleSolenoidItem::DoubleSolenoidItem() {}

DoubleSolenoidItem::DoubleSolenoidItem(string _name, int _forwardChannel, int _reverseChannel, DoubleSolenoid::Value _default, bool _reversed)
	: OutputComponent(_name){
	forwardChannel = _forwardChannel;
	reverseChannel = _reverseChannel;
	this->_default = _default;
	reversed = _reversed;
	DefaultSet();
	solenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
}

void DoubleSolenoidItem::DefaultSet(){
	Set(_default);
}

void DoubleSolenoidItem::Set(DoubleSolenoid::Value value){
	if(!inUse){
		inUse = true;
		switch(value){
			case DoubleSolenoid::Value::kForward:
				SetForward();
				break;
			case DoubleSolenoid::Value::kReverse:
				SetReverse();
				break;
			case DoubleSolenoid::Value::kOff:
				SetOff();
				break;
		}
		inUse = false;
	}
}

double DoubleSolenoidItem::Get()
{
	double ret = 0.0;
	DoubleSolenoid::Value curState = GetState();
	switch(curState)
	{
		case DoubleSolenoid::Value::kOff : return 0.0;
		case DoubleSolenoid::Value::kForward : return 1.0;
		case DoubleSolenoid::Value::kReverse : return 2.0;
	}	
	return ret;
}

DoubleSolenoid::Value DoubleSolenoidItem::GetState(){
	return solenoid->Get();
}

DoubleSolenoid::Value DoubleSolenoidItem::GetDefaultValue(){
	return _default;
}

void DoubleSolenoidItem::Set(double value){
	if(!inUse){
		inUse = true;
		if (value >= -1 && value <= 1)
		{
			 if (abs(value + 1) < EPSILON_MIN)
				SetOff();
			else if (abs(value) < EPSILON_MIN)
			{
				if (!reversed)
					SetForward();
				else
					SetReverse();
			}
			else if (abs(value - 1) < EPSILON_MIN)
			{
				if (!reversed)
					SetReverse();
				else
					SetForward();
			}
		}
		else
		{
			Log::Error("The valid arguments for DoubleSolenoid " + name + " is Reverse, Off, and Forward (-1, 0, 1). I can't tell you who sent the screwy command, because C++ doesn't do object sender...");
		}
		inUse = false;
	}
}

void DoubleSolenoidItem::Set(bool value){
	Set((double)value);
}

void DoubleSolenoidItem::SetForward(){
	solenoid->Set(DoubleSolenoid::Value::kForward);
}

void DoubleSolenoidItem::SetReverse(){
	solenoid->Set(DoubleSolenoid::Value::kReverse);
}

void DoubleSolenoidItem::SetOff(){
	solenoid->Set(DoubleSolenoid::Value::kOff);
}

DoubleSolenoidItem::~DoubleSolenoidItem() {}

