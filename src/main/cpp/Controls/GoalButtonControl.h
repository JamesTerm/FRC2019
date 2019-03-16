/****************************** Header ******************************\
Class Name:	GoalButtonControl inherits ControlItem
File Name:	GoalButtonControl.h
Summary:	Interface for a single button control that fires a goal.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson
Email:	dylantrwatson@gmail.com
\*********************************************************************/

#pragma once

#include "ControlItem.h"
#include "../Components/VictorSPItem.h"
#include "../Components/DoubleSolenoidItem.h"
#include "../Goals/FRC2019_Goals.h"
#include "../Goals/GoalSelector.h"

namespace Controls{

class GoalButtonControl : public ControlItem {
private:
	int m_button;
	double m_current;
	double m_previous;

public:
	GoalButtonControl();
	GoalButtonControl(Joystick *_joy, string _name, int _button, ActiveCollection* ac, TeleOpGoal _goal, double _target, bool _reversed = false, double _powerMultiplier = 1.0);
	virtual ~GoalButtonControl();
	int getSign(double val);
	virtual double Update(double _dTime) override;

	TeleOpGoal m_goal;
	double m_target;
	bool m_goalSet = false;
};
}