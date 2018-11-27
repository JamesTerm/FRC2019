/****************************** Header ******************************\
Class Name: PotentiometerItem inherits InputComponent
File Name:	PotentiometerItem.cpp
Summary: Abstraction for the WPIlib AnalogPotentiometer that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylan.watson@broncbotz.org
\********************************************************************/

#include <PotentiometerItem.h>

using namespace Components;

PotentiometerItem::PotentiometerItem() {}

PotentiometerItem::PotentiometerItem(int _channel, string _name)
	: InputComponent(_name){
	channel = _channel;
	apt = new AnalogPotentiometer(channel);
}

string PotentiometerItem::GetName(){
	return name;
}

double PotentiometerItem::Get(){
	return apt->Get();
}

PotentiometerItem::~PotentiometerItem() {}

