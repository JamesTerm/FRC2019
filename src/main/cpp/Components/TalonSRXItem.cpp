/****************************** Header ******************************\
Class Name: TalonSRXItem inherits OutputComponent
File Name:	TalonSRXItem.cpp
Summary: Abstraction for the WPIlib TalonSRX that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#include <iostream>

#include "TalonSRXItem.h"

using namespace std;
using namespace frc;
using namespace Components;

TalonSRXItem::TalonSRXItem() {}

TalonSRXItem::TalonSRXItem(int _channel, string _name, bool _reversed, bool enableEncoder)
	: Motor(_name){
	channel = _channel;
	reversed = _reversed;
	talon = new TalonSRX(channel);
	encoderEnabled = enableEncoder;
	if(enableEncoder)
		talon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
}

double TalonSRXItem::Get(){
    if (reversed)
        return talon->GetMotorOutputPercent() * -1;
    return talon->GetMotorOutputPercent();
}

int TalonSRXItem::GetQuadraturePosition(){
	return encoderEnabled ? talon->GetSensorCollection().GetQuadraturePosition() : -1;
}

void TalonSRXItem::SetQuadraturePosition(int val){
	talon->GetSensorCollection().SetQuadraturePosition(val, 10);
}

void TalonSRXItem::Set(double val){
	val = CalculateVal(val);
	Log::General(name+" : "  + to_string(val));
	if((val<0 || val>0) && !inUse)
	{
		inUse = true;
		if(reversed) talon->Set(ControlMode::PercentOutput, -val);
		else talon->Set(ControlMode::PercentOutput, val);
		inUse = false;
	}
	else if(!inUse){
		inUse = true;
		talon->Set(ControlMode::PercentOutput, 0);
		inUse = false;
	}
}

void TalonSRXItem::DefaultSet(){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void TalonSRXItem::Set(DoubleSolenoid::Value value){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void TalonSRXItem::Stop()
{
	TalonSRXItem::Set(0);
}

void TalonSRXItem::DeleteComponent()
{
	delete talon;
	delete this;
}

TalonSRXItem::~TalonSRXItem() {}

