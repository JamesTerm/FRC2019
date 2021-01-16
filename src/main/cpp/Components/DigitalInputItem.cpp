/****************************** Header ******************************\
Class Name: DigitalInputItem inherits InputComponent
File Name:	DigitalInputItem.cpp
Summary: Abstraction for the WPIlib DigitalInput that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#include "DigitalInputItem.h"

using namespace Components;

DigitalInputItem::DigitalInputItem(int _channel, string name) : InputComponent(name)
{
	channel = _channel;
	din = new DigitalInput(channel);

}

void DigitalInputItem::DeleteComponent()
{
	delete din;
	delete this;
}

double DigitalInputItem::Get()
{
	return din->Get();
}

bool DigitalInputItem::GetBool()
{
	//This is bad... no since in casting to a double and back into an int, and not all control paths return a value
#if 0
	int get = Get();
	if(get == 1)
		return false;
	else if(get == 0)
		return true;
#else
	//This follows the logic previously, and for some unknown value... its false
	return din->Get() == 0;
#endif
}

string DigitalInputItem::GetName()
{
	return name;
}

DigitalInputItem::~DigitalInputItem() 
{
}

