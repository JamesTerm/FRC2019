/****************************** Header ******************************\
Class Name:	ControlItem
File Name:	ControlItem.cpp
Summary:	Abstraction for managing all driver and operator controls
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson
Email:	dylantrwatson@gmail.com
\*********************************************************************/

#include <iostream>

#include "ControlItem.h"

using namespace std;
using namespace Controls;

#pragma region Event Handlers

auto onControllerValueChanged = [&](EventArgs* e) {
	try{
		auto args = (TEventArgs<double, ControlItem*>*)e;
		args->GetSender()->SetToComponents(args->GetValue());
		delete args;
		args = nullptr;
	}catch(exception &e){
		Log::Error("Known Exception Thrown in onControllerValueChanged in a Control! This can cause fatal Runtime Errors! Check your logs and XML.");
		//TODO: Make this the append instead
		Log::Error(e.what());
	}catch(...){
		Log::Error("UnknownException Thrown in onControllerValueChanged in a Control! This can cause fatal Runtime Errors! Check your XML and yell at the programmers!");
	}
};

#pragma endregion

ControlItem::ControlItem(){}

ControlItem::ControlItem(Joystick *_joy, string _name, bool _reversed, double _powerMultiplier)
{
	joy = _joy;
	name = _name;
	reversed = _reversed;
	powerMultiplier = _powerMultiplier;
	ValueChanged += onControllerValueChanged;
}

void ControlItem::AddComponent(OutputComponent *component)
{
	components.push_back(component);
}

vector<string> ControlItem::GetComponents()
{
	vector<string> componentNames;
	for(int i=0; i<(int)components.size();i++)
		componentNames.push_back((*components[i]).name);
	return componentNames;
}

void ControlItem::SetToComponents(double val)
{
	try
	{
		for(int i=0; i<(int)components.size();i++)
			(*components[i]).Set(val);
	}
	catch(...)
	{
		cout << "Error setting value to binding for " << name << " control!" << endl;
	}
}

ControlItem::~ControlItem() {}

