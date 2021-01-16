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

EncoderItem::EncoderItem(string _name, int _aChannel, int _bChannel, bool _reversed)
	: InputComponent(_name){
	aChannel = _aChannel;
	bChannel = _bChannel;
	reversed = _reversed;
	encoder = new Encoder(aChannel, bChannel, reversed);
}

void EncoderItem::Reset(){
	encoder->Reset();
}

double EncoderItem::Get(){
	double input = ((double)encoder->Get());
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