/****************************** Header ******************************\
Class Name:	ControlItem
File Name:	ControlItem.cpp
Summary:	Abstraction for managing all driver and operator controls
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson
Email:	dylantrwatson@gmail.com
\*********************************************************************/

#include <iostream>

#include "ControlItem.h"
#include "GoalButtonControl.h"
#include "SwerveControl.h"
#include "../Goals/GoalSelector.h"
#include "../Goals/FRC2019_Goals.h"

using namespace std;
using namespace Controls;

#pragma region Event Handlers

auto onControllerValueChanged = [&](EventArgs* e) {
	try{
		auto args = (TEventArgs<double, ControlItem*>*)e;
	Log::General(" EVENT "+args->GetSender()->name);

		//If the sender is a GoalButtonControl
		if (dynamic_cast<GoalButtonControl*>(args->GetSender())) {
			//Reset the active TeleOpGoal
			args->GetSender()->m_activeCollection->GetActiveGoal()->Reset();
			TeleOpGoal goal = ((GoalButtonControl*)(args->GetSender()))->m_goal;
			SmartDashboard::PutString("GoalButtonControl Status", "Control Found");
			//Convert from TeleOpGoal[enum] to a Goal[class]
			Goal* goalToAdd = SelectTeleOpGoal(args->GetSender()->m_activeCollection, goal, args->GetValue());
			MultitaskGoal* teleOpMasterGoal = args->GetSender()->m_activeCollection->GetActiveGoal();
			//Add in a timeout and ControllerOverride to the the Multitask Goal
			//TODO: HOLY CRAP REENABLE THIS
			//teleOpMasterGoal->AddGoal(new Goal_TimeOut(args->GetSender()->m_activeCollection, 7));
			teleOpMasterGoal->AddGoal(new Goal_ControllerOverride(args->GetSender()->m_activeCollection, 1));
			teleOpMasterGoal->AddGoal(goalToAdd);
			//Set and activate the TeleOp Goal
			args->GetSender()->m_activeCollection->SetActiveGoal(teleOpMasterGoal);
			args->GetSender()->m_activeCollection->GetActiveGoal()->Activate();
			((GoalButtonControl*)(args->GetSender()))->m_goalSet = true;
			SmartDashboard::PutString("GoalButtonControl Status", "GoalActivated");
			return;
		}
		args->GetSender()->SetToComponents(args->GetValue());
		SmartDashboard::PutNumber(args->GetSender()->name, args->GetValue());
	}catch(...){
		try{
			auto SwerveArgs = (IEventArgs<double, double, double, ControlItem*>*)e;
			Log::General(" EVENT "+SwerveArgs->GetSender()->name);

			SwerveControl *DT = (SwerveControl*)(SwerveArgs->GetSender());
			DT->GetManager()->Set(SwerveArgs->GetVValue(), SwerveArgs->GetHValue(), SwerveArgs->GetSValue());

			SmartDashboard::PutNumber(SwerveArgs->GetSender()->name, SwerveArgs->GetVValue());
		}
		catch(exception &e)
		{
			Log::Error("Known Exception Thrown in onControllerValueChanged in a Control! This can cause fatal Runtime Errors! Check your logs and XML.");
			SmartDashboard::PutString("OnValueChangedStatus", "Error");
			//TODO: Make this the append instead
			Log::Error(e.what());
		}
		catch(...){
			Log::Error("UnknownException Thrown in onControllerValueChanged in a Control! This can cause fatal Runtime Errors! Check your XML and yell at the programmers!");
			SmartDashboard::PutString("OnValueChangedStatus", "Error");
		}
	}
};

#pragma endregion

ControlItem::ControlItem(){}

ControlItem::ControlItem(Joystick *_joy, string _name, bool _reversed, double _powerMultiplier, ActiveCollection* activeCollection)
{
	m_activeCollection = activeCollection;
	joy = _joy;
	name = _name;
	reversed = _reversed;
	powerMultiplier = _powerMultiplier;
	ValueChanged += onControllerValueChanged;
}

void ControlItem::AddComponent(OutputComponent *component)
{
	components.push_back(component);
}

vector<string> ControlItem::GetComponents()
{
	vector<string> componentNames;
	for(int i=0; i<(int)components.size();i++)
		componentNames.push_back((*components[i]).name);
	return componentNames;
}

void ControlItem::SetToComponents(double val)
{
	try
	{
		for(int i=0; i<(int)components.size();i++)
			(*components[i]).Set(val);
	}
	catch(...)
	{
		Log::Error("Error setting value to binding for " + name + " control!");
	}
}

void ControlItem::DeleteComponent()
{
	delete this;
}

ControlItem::~ControlItem() {}

