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
	double m_current = 0;
	double m_previous = 0;

public:
	GoalButtonControl();
	GoalButtonControl(Joystick *_joy, string _name, int _button, ActiveCollection* ac, TeleOpGoal _goal, double _target, int KeyID, vector<int> RemoveKeys, bool _reversed = false, double _powerMultiplier = 1.0);
	virtual ~GoalButtonControl();
	int getSign(double val);
	virtual double Update(double _dTime) override;

	TeleOpGoal m_goal;
	double m_target;
	bool m_goalSet = false;
	
	int IdKey = 0;
	vector<int> RemoveKeys;
};
}