/****************************** Header ******************************\
Class Name:	GoalAxisControl inherits ControlItem
File Name:	GoalAxisControl.cpp
Summary:	Interface for a multiple axis control.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Ian Poll
Email:	irobot9803@gmail.com
\*********************************************************************/

#include "GoalAxisControl.h"
#include <cmath>

using namespace Controls;

GoalAxisControl::GoalAxisControl() {}

GoalAxisControl::GoalAxisControl(Joystick *_joy, string _name, vector<int> Axis, ActiveCollection* ac, TeleOpGoal _goal, vector<string> Strings, int _StartIndex, int KeyID, vector<int> _RemoveKeys, bool _RepeatWhenFinished, double DeadZone, double _powerMultiplier) : ControlItem(_joy, _name, false, _powerMultiplier, ac)
{
	m_Axis = Axis;
	m_goal = _goal;
    m_StringVals = Strings;
    DeadZ = DeadZone;
    StartIndex = _StartIndex;
    IdKey = KeyID;
	RemoveKeys = _RemoveKeys;
    Mult = _powerMultiplier;
    RepeatWhenFinished = _RepeatWhenFinished;
}

double GoalAxisControl::Update(double _dTime)
{
    bool Out = false;
    double Average = 0;
    m_AxisVals.clear();

    for(int i = 0; i < m_Axis.size(); i++)
    {
        m_AxisVals.push_back((m_Axis[i] >= 0 ? 1 : -1) * joy->GetRawAxis(abs(m_Axis[i])) * Mult);
        Average += joy->GetRawAxis(m_Axis[i]);
        if(abs(joy->GetRawAxis(m_Axis[i])) > DeadZ)
        {
            Out = true;
        }
    }
    Average /= m_Axis.size();
    
    if(Out)
    {
        ValueChanged(new LEventArgs<vector<double>, vector<string>, GoalAxisControl*>(m_AxisVals, m_StringVals, this));
    }
    
    return Average;
}

GoalAxisControl::~GoalAxisControl() {}