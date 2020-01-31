/****************************** Header ******************************\
Class Name: SparkMax inherits OutputComponent
File Name:	SparkMaxItem.cpp
Summary: Abstraction for the SparkMax
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot983@gmail.com
\********************************************************************/

#include <iostream>

#include "SparkMaxItem.h"

using namespace std;
using namespace frc;
using namespace Components;

SparkMaxItem::SparkMaxItem() {}

SparkMaxItem::SparkMaxItem(int _channel, string _name, bool _reversed)
	: OutputComponent(_name){
	channel = _channel;
	reversed = _reversed;
	Max = new CANSparkMax(channel);
	Encoder = new CANEncoder (Max, CANEncoder::kQuadrature);
}
double SparkMaxItem::Get(){
    return Max->GetAppliedOutput();
}
double SparkMaxItem:: GetEncoderValue(){
	return Encoder -> GetPosition();
}
void SparkMaxItem::Set(double val){
	
	if((val<0 || val>0) && !inUse)
	{
		inUse = false;
		if(reversed) Max->Set(-val);
		else Max->Set(val);
		inUse = false;
	}
	else if(!inUse)
	{
		inUse = true;
		Max->Set(0);
		inUse = false;
	}
}

void SparkMaxItem::SetPDBChannel(int val){
	PDBChannel = val;
}

void SparkMaxItem::DefaultSet(){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers! Retard (or don't, its really up to you)");
}

void SparkMaxItem::Set(DoubleSolenoid::Value value){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers! Retard (or don't, its really up to you)");
}

SparkMaxItem::~SparkMaxItem() {}
