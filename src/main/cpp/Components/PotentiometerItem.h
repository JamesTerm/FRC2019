/****************************** Header ******************************\
Class Name: PotentiometerItem inherits InputComponent
File Name:	PotentiometerItem.h
Summary: Abstraction for the WPIlib AnalogPotentiometer that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#ifndef SRC_COMPONENTS_POTENTIOMETERITEM_H_
#define SRC_COMPONENTS_POTENTIOMETERITEM_H_

#include "InputComponent.h"

namespace Components{
class PotentiometerItem : public InputComponent{
private:
	int channel;
	int initPosition; //this is set on init. Ensures accurate measurement of distance traveled. As long as elevator is all the way down on startup, which it should
	AnalogPotentiometer *apt;

public:
	PotentiometerItem();
	PotentiometerItem(int _channel, string _name, bool Real);
	virtual string GetName() override;
	virtual double Get() override;
	virtual void DeleteComponent() override;
	virtual void UpdateComponent() override;
	virtual ~PotentiometerItem();
};
}

#endif /* SRC_COMPONENTS_POTENTIOMETERITEM_H_ */
