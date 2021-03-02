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
	Offset = OutputTable->GetNumber(name, 0);
	{
		Log::General("Using Table values");
		OutputTable->PutNumber(name, 0);
		OutputTable->PutBoolean(name + "-Reset", true);
	}
	Type = InputType::Independent;
}

EncoderItem::EncoderItem(string _name, NativeComponent *Connected) : InputComponent(_name)
{
	Type = InputType::Data_Driven;
	LinkedComponent = Connected;
}

void EncoderItem::Reset()
{
	if(Type == InputType::Independent)
	{
		encoder->Reset();
		OutputTable->PutBoolean(name + "-Reset", true);
	}
	else
	{
		if(LinkedComponent != nullptr)
			LinkedComponent->ResetData();
		else
			Log::Error("Encoder " + name + " tracking nullptr!");
	}
}

double EncoderItem::Get()
{
	if(Type == InputType::Independent)
	{
		double input = (UseTable ? OutputTable->GetNumber(name, 0) : (double)encoder->Get());
		return input - Offset;
	}
	else
	{
		if(LinkedComponent != nullptr)
			return LinkedComponent->GetData();
		else
		{
			Log::Error("Encoder " + name + " tracking nullptr!");
			return 0;
		}
	}
}

string EncoderItem::GetName()
{
	return name;
}

void EncoderItem::DeleteComponent()
{
	delete encoder;
	delete this;
}

void EncoderItem::UpdateComponent()
{
	if (!UseTable && Type == InputType::Independent)
	{
		OutputTable->PutNumber(name, EncoderItem::Get());
	}
}

EncoderItem::~EncoderItem() {}