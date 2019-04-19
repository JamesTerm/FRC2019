/****************************** Header ******************************\
Class Name: PotentiometerItem inherits InputComponent
File Name:	PotentiometerItem.cpp
Summary: Abstraction for the WPIlib AnalogPotentiometer that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#include "PotentiometerItem.h"

using namespace Components;

PotentiometerItem::PotentiometerItem() {}

PotentiometerItem::PotentiometerItem(int _channel, string _name)
	: InputComponent(_name){
	channel = _channel;
	apt = new AnalogPotentiometer(channel);
	initPosition = apt->Get();
}

string PotentiometerItem::GetName(){
	return name;
}

double PotentiometerItem::Get(){
	Log::General("real: " + to_string(apt->Get()));
	return apt->Get() - initPosition; //init position it subtracted to return the delta from startup position.
}

PotentiometerItem::~PotentiometerItem() {}