/****************************** Header ******************************\
Class Name: DoubleSolenoidItem inherits OutputComponent
File Name:	DoubleSolenoidItem.h
Summary: Abstraction for the WPIlib DoubleSolenoid that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#ifndef SRC_COMPONENTS_DOUBLESOLENOIDITEM_H_
#define SRC_COMPONENTS_DOUBLESOLENOIDITEM_H_

#include "OutputComponent.h"

using namespace std;

namespace Components{
class DoubleSolenoidItem : public OutputComponent{
private:
	int forwardChannel;
	int reverseChannel;
	DoubleSolenoid::Value _default;
	bool reversed;
	DoubleSolenoid *solenoid;

public:
	DoubleSolenoidItem();
	DoubleSolenoidItem(string _name, int _forwardChannel, int _reverseChannel, DoubleSolenoid::Value _default, bool _reversed, bool Real);
	virtual void DefaultSet() override;
	virtual void Set(DoubleSolenoid::Value value) override;
	void Set(bool value);
	void SetForward();
	void SetReverse();
	void SetOff();
	virtual void Set(double value) override;
	virtual void DeleteComponent() override;
	virtual double Get() override;
	DoubleSolenoid::Value GetState();
	DoubleSolenoid::Value GetDefaultValue();
	virtual ~DoubleSolenoidItem();
	DoubleSolenoid *AsDoubleSolenoid() { return solenoid; }
};
}

#endif /* SRC_COMPONENTS_DOUBLESOLENOIDITEM_H_ */
