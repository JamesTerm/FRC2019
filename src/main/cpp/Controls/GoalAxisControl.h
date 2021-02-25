/****************************** Header ******************************\
Class Name:	GoalAxisControl inherits ControlItem
File Name:	GoalAxisControl.h
Summary:	Interface for a multiple axis control that fires a goal.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Ian Poll
Email:	irobot9803@gmail.com
\*********************************************************************/

#pragma once

#include "ControlItem.h"
#include "../Goals/FRC2019_Goals.h"
#include "../Goals/GoalSelector.h"

namespace Controls{

class GoalAxisControl : public ControlItem {
private:
	vector<int> m_Axis;
    vector<double> m_AxisVals;
	vector<string> m_StringVals;
    
	double m_current = 0;
	double m_previous = 0;
    double DeadZ = 0.001;
	double Mult = 1;

public:
	GoalAxisControl();
	GoalAxisControl(Joystick *_joy, string _name, vector<int> Axis, ActiveCollection* ac, TeleOpGoal _goal, vector<string> Strings, int _StartIndex, int KeyID, vector<int> RemoveKeys, bool _RepeatWhenFinished = false, double DeadZone = 0.01, double _powerMultiplier = 1.0);
	virtual ~GoalAxisControl();
	virtual double Update(double _dTime) override;

	TeleOpGoal m_goal;
	int StartIndex = 0;
	
	int IdKey = 0;
	vector<int> RemoveKeys;
	bool RepeatWhenFinished;
};
}