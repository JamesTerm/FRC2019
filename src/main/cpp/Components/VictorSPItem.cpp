/****************************** Header ******************************\
Class Name: VictorSPItem inherits OutputComponent
File Name:	VictorSPItem.cpp
Summary: Abstraction for the WPIlib VictorSP that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#include <iostream>

#include "VictorSPItem.h"

using namespace std;
using namespace Components;

void VictorSPItem::Set(double val)
{
	val = CalculateVal(val);
	if((val<0 || val>0) && !inUse)
	{
		inUse = true;
		if(Reversed) victor->Set(-val);
		else victor->Set(val);
		inUse = false;
	}
	else if(!inUse)
	{
		inUse = true;
		victor->Set(0);
		inUse = false;
	}
}

void VictorSPItem::Stop()
{
	if(!inUse)
	{
		inUse = true;
		victor->Set(0);
		inUse = false;
	}
}

double VictorSPItem::Get()
{
	if(Reversed)
		return victor->Get() * -1;
	return victor->Get();
}

string VictorSPItem::GetName()
{
	return name;
}

int VictorSPItem::GetPolarity()
{
	if(Reversed) return -1;
	return 1;
}

void VictorSPItem::DeleteComponent()
{
	CleanUpProfiles();
	delete victor;
	delete this;
}

void VictorSPItem::DefaultSet()
{
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void VictorSPItem::Set(DoubleSolenoid::Value value)
{
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}
