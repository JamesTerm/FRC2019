/****************************** Header ******************************\
Class Name: EncoderItem inherits InputComponent
File Name:	EncoderItem.cpp
Summary: Abstraction for the WPIlib Encoder that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#include "EncoderItem.h"

using namespace Components;

EncoderItem::EncoderItem() {}

EncoderItem::EncoderItem(string _name, int _aChannel, int _bChannel, bool _reversed, bool Real)
	: InputComponent(_name){
	aChannel = _aChannel;
	bChannel = _bChannel;
	reversed = _reversed;
	encoder = new Encoder(aChannel, bChannel, reversed);
	FromTable(Real);
	if (UseTable)
	{
		Log::General("Using Table values");
		OutputTable->PutNumber(name, 0);
		OutputTable->PutBoolean(name + "-Reset", true);
	}
}

void EncoderItem::Reset(){
	encoder->Reset();
	if (UseTable)
		OutputTable->PutBoolean(name + "-Reset", true);
}

double EncoderItem::Get(){
	double input = (UseTable ? OutputTable->GetNumber(name, 0) : (double)encoder->Get());
	return input;
}

string EncoderItem::GetName(){
	return name;
}

void EncoderItem::DeleteComponent()
{
	delete encoder;
	delete this;
}

EncoderItem::~EncoderItem() {}