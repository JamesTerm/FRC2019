/****************************** Header ******************************\
Class Name:	Drive
File Name:	Drive.cpp
Summary: 	Drive loop
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Ryan Cooper, Dylan Watson
Email:	cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com
\*********************************************************************/

#include "Drive.h"
#include <iostream>

using namespace std;
using namespace System;

Drive::Drive(ActiveCollection* activeCollection) { 
	m_activeCollection = activeCollection;
	m_teleOpGoal = new MultitaskGoal(m_activeCollection);
}

void Drive::AddControlDrive(ControlItem *control)
{
	m_driveControlCollection.push_back(control);
}

void Drive::AddControlOperate(ControlItem *control)
{
	m_operateControlCollection.push_back(control);
}

void Drive::Update(double deltaTime)
{
	if (!m_DisableDrive)
	{
		for (int i = 0; i < (int)m_driveControlCollection.size(); i++)
			(*m_driveControlCollection[i]).Update(deltaTime);
	}
	if (!m_DisableOperator)
	{
		for (int i = 0; i < (int)m_operateControlCollection.size(); i++)
			(*m_operateControlCollection[i]).Update(deltaTime);
	}
}