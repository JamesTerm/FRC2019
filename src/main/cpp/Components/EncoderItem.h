/****************************** Header ******************************\
Class Name: EncoderItem inherits InputComponent
File Name:	EncoderItem.h
Summary: Abstraction for the WPIlib Encoder that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/
#ifndef SRC_COMPONENTS_ENCODERITEM_H_
#define SRC_COMPONENTS_ENCODERITEM_H_

#include "InputComponent.h"

using namespace std;
using namespace frc;
using namespace Util;

namespace Components{
class EncoderItem : public InputComponent{
private:
	enum InputType
	{
		Independent = 0,
		Data_Driven = 1
	};

	int aChannel;
	int bChannel;
	bool reversed;
	int Offset = 0;
	Encoder *encoder = nullptr;
	NativeComponent* LinkedComponent = nullptr;
	
	InputType Type;

public:

	virtual double Get() override;
	EncoderItem();
	EncoderItem(string _name, int _aChannel, int _bChannel, bool _reversed, bool Real);
	EncoderItem(string _name, NativeComponent *Connected);
	NativeComponent* GetLinkedComponent(){return LinkedComponent;};
	virtual string GetName() override;
	void Reset();
	virtual void DeleteComponent() override;
	virtual void UpdateComponent() override;
	virtual ~EncoderItem();
};
}

#endif /* SRC_COMPONENTS_ENCODERITEM_H_ */
