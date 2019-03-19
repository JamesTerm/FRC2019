/****************************** Header ******************************\
Class Name:	ButtonControl inherits ControlItem
File Name:	ButtonControl.cpp
Summary:	Interface for a single button control.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson
Email:	dylantrwatson@gmail.com
\*********************************************************************/

#include "GoalButtonControl.h"

using namespace Controls;

GoalButtonControl::GoalButtonControl() {}

GoalButtonControl::GoalButtonControl(Joystick *_joy, string _name, int _button, ActiveCollection* ac, TeleOpGoal _goal, double _target, bool _reversed, double _powerMultiplier)
	: ControlItem(_joy, _name, _reversed, _powerMultiplier, ac) {
	m_button = _button;
	m_goal = _goal;
	m_target = _target;
}

double GoalButtonControl::Update(double _dTime){
	double val = joy->GetRawButton(m_button); //*< Value recieved from the button
	
	m_current = val;

	if (!val && abs(m_previous - m_current) > EPSILON_MIN)
		ValueChanged(new TEventArgs<double, GoalButtonControl*>(m_target, this));
	if (m_activeCollection->GetActiveGoal()->GetStatus() == Goal::eActive && m_goalSet) {
		m_activeCollection->GetActiveGoal()->Process(0.010);
	}
	else if (!(m_activeCollection->GetActiveGoal()->GetStatus() == Goal::eActive) && m_goalSet) {
		m_goalSet = false;
		m_activeCollection->GetActiveGoal()->Reset();
	}
	m_previous = m_current;
	return m_current;
}

GoalButtonControl::~GoalButtonControl() {}