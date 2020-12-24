/****************************** Header ******************************\
Class Name: VictorSPXItem inherits OutputComponent
File Name:	VictorSPXItem.cpp
Summary: Abstraction for the VictorSPX
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot983@gmail.com
\********************************************************************/

#include <iostream>

#include "VictorSPXItem.h"

using namespace std;
using namespace frc;
using namespace Components;

VictorSPXItem::VictorSPXItem() {}

VictorSPXItem::VictorSPXItem(int _channel, string _name, bool _reversed)
	: Motor(_name){
	channel = _channel;
	reversed = _reversed;
	victor = new VictorSPX(channel);
	victor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
}

double VictorSPXItem::Get(){
    if (reversed)
        return victor->GetMotorOutputPercent() * -1;
    return victor->GetMotorOutputPercent();
}

void VictorSPXItem::Set(double val){
	val = CalculateVal(val);
	if((val<0 || val>0) && !inUse)
	{
		inUse = true;
		if(reversed) victor->Set(ControlMode::PercentOutput, -val);
		else victor->Set(ControlMode::PercentOutput, val);
		inUse = false;
	}
	else if(!inUse){
		inUse = true;
		victor->Set(ControlMode::PercentOutput, 0);
		inUse = false;
	}
}

void VictorSPXItem::DefaultSet(){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void VictorSPXItem::Set(DoubleSolenoid::Value value){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void VictorSPXItem::Stop()
{
	VictorSPXItem::Set(0);
}

VictorSPXItem::~VictorSPXItem() {}
