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

namespace Components{
class EncoderItem : public InputComponent{
private:
	int aChannel;
	int bChannel;
	bool reversed;
	Encoder *encoder;

public:
	virtual double Get() override;
	EncoderItem();
	EncoderItem(string _name, int _aChannel, int _bChannel, bool _reversed, bool Real);
	virtual string GetName() override;
	void Reset();
	virtual void DeleteComponent() override;
	virtual ~EncoderItem();
};
}

#endif /* SRC_COMPONENTS_ENCODERITEM_H_ */
