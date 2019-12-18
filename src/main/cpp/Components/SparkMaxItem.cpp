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
	Max = new SparkMax(channel);
}

double SparkMaxItem::Get(){
    return 0;
}

void SparkMaxItem::Set(double val){
	
}

void SparkMaxItem::SetPDBChannel(int val){
	PDBChannel = val;
}

void SparkMaxItem::DefaultSet(){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers! (or don't, its really up to you)");
}

void SparkMaxItem::Set(DoubleSolenoid::Value value){
	Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers! (or don't, its really up to you)");
}

SparkMaxItem::~SparkMaxItem() {}
