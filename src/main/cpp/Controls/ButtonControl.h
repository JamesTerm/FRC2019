/****************************** Header ******************************\
Class Name:	ButtonControl inherits ControlItem
File Name:	ButtonControl.h
Summary:	Interface for a single button control.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson
Email:	dylantrwatson@gmail.com
\*********************************************************************/

#ifndef SRC_CONTROLS_BUTTONCONTROL_H_
#define SRC_CONTROLS_BUTTONCONTROL_H_

#include "ControlItem.h"
#include "../Components/VictorSPItem.h"
#include "../Components/DoubleSolenoidItem.h"

namespace Controls{
class ButtonControl : public ControlItem {
private:
	int button;
	bool actOnRelease;
	double previous;
	double current;
	bool isSolenoid;
	bool isRamp;
	double inc;
	int powerPort;
	bool isAmpRegulated;
	double ampLimit;
	PowerDistributionPanel *pdp;

	bool isOverdrive;

public:
	ButtonControl();
	ButtonControl(Joystick *_joy, string _name, int _button, bool _actOnRelease, bool _reversed, double _powerMultiplier, bool _isSolenoid, ActiveCollection* ac, bool _isOverdrive = false);
	void SetToSolenoids(DoubleSolenoid::Value value);
	void SetSolenoidDefault();
	void SetRamp(double _inc);
	void SetAmpRegulation(int _powerPort, double _ampLimit);
	virtual ~ButtonControl();
	int getSign(double val);
	virtual void DeleteComponent() override;
	virtual double Update(double _dTime) override;
};
}

#endif /* SRC_CONTROLS_BUTTONCONTROL_H_ */
