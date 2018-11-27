/****************************** Header ******************************\
Class Name: PotentiometerItem inherits InputComponent
File Name:	PotentiometerItem.h
Summary: Abstraction for the WPIlib AnalogPotentiometer that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylan.watson@broncbotz.org
\********************************************************************/

#ifndef SRC_COMPONENTS_POTENTIOMETERITEM_H_
#define SRC_COMPONENTS_POTENTIOMETERITEM_H_

#include "WPILib.h"
#include "InputComponent.h"

namespace Components{
class PotentiometerItem : public InputComponent{
private:
	int channel;
	AnalogPotentiometer *apt;

public:
	PotentiometerItem();
	PotentiometerItem(int _channel, string _name);
	virtual string GetName() override;
	virtual double Get() override;
	virtual ~PotentiometerItem();
};
}

#endif /* SRC_COMPONENTS_POTENTIOMETERITEM_H_ */
