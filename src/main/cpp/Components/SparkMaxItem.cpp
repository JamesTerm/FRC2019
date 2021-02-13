/****************************** Header ******************************\
Class Name: SparkMax inherits OutputComponent
File Name:	SparkMaxItem.cpp
Summary: Abstraction for the SparkMax
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll, Shruti Venkatramanan, Guadalupe Rodriguez, Emily Martinez
Email: irobot983@gmail.com
\********************************************************************/

#include <iostream>

#include "SparkMaxItem.h"

using namespace std;
using namespace frc;
using namespace Components;

SparkMaxItem::SparkMaxItem() {}

SparkMaxItem::SparkMaxItem(int _channel, string _name, bool _reversed, bool Real) : Motor(_name){
	channel = _channel;
	reversed = _reversed;
	Max = new CANSparkMax(channel, rev::CANSparkMax::MotorType::kBrushless);
	Max->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	Name = _name;
	Offset = 0;
	FromTable(Real);
	if (UseTable)
	{
		Log::General("Using Table values");
		OutputTable->PutNumber(name + "-Encoder", 0);
		OutputTable->PutBoolean(name + "-Reset", true);
	}
}

double SparkMaxItem::Get(){
    return Max->Get();
}

double SparkMaxItem:: GetEncoderValue(){
	return (UseTable ? OutputTable->GetNumber(name + "-Encoder", 0) : (Max->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, EncTicks).GetPosition() - Offset));
}

void SparkMaxItem::Reset(){
	Offset = Max->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, EncTicks).GetPosition();
	if (UseTable)
		OutputTable->PutBoolean(name + "-Reset", true);
}

string SparkMaxItem::GetName(){
	return Name;
}

int SparkMaxItem::GetPolarity(){
	return (reversed? -1 : 1);
}

void SparkMaxItem::Set(double val){
	val = CalculateVal(val);
	// Log::General(Name+" : "  + to_string(val));
	if((val<0 || val>0) && !inUse)
	{
		inUse = true;
		if(reversed) Max->Set(-val);
		else Max->Set(val);
		inUse = false;
	}
	else if(!inUse)
	{
		inUse = true;
		Max->StopMotor();
		inUse = false;
	}
}

void SparkMaxItem::Stop(){
	if(!inUse)
	{
		inUse = true;
		Max->StopMotor();
		inUse = false;
	}
}

void SparkMaxItem::DeleteComponent()
{
	delete Max;
	delete this;
}

void SparkMaxItem::UpdateComponent()
{
	if (!UseTable)
	{
		OutputTable->PutNumber(name + "-Encoder", SparkMaxItem::GetEncoderValue());
	}
}

void SparkMaxItem::ResetEncoderValue(){
	Max->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 24).SetPosition(0);

}

void SparkMaxItem::DefaultSet(){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers! Retard (or don't, its really up to you)");
}

void SparkMaxItem::Set(DoubleSolenoid::Value value){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers! Retard (or don't, its really up to you)");
}

SparkMaxItem::~SparkMaxItem() {}
