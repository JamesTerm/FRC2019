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
#include "GoalAxisControl.h"
#include "SwerveControl.h"
#include "../Goals/GoalSelector.h"
#include "../Goals/FRC2019_Goals.h"

using namespace std;
using namespace Controls;

#pragma region Event Handlers

auto onControllerValueChanged = [&](EventArgs* e) {
	try{
		if (e->Type == 0)
		{
			auto args = (TEventArgs<double, ControlItem*>*)e;
			Log::General(" EVENT "+args->GetSender()->name);

			//If the sender is a GoalButtonControl
			if (dynamic_cast<GoalButtonControl*>(args->GetSender())) {

				MultitaskGoal* Selected = nullptr;

				if(args->GetSender()->Holder == ControlItem::JoystickHolder::DriverController)
				{
					Selected = args->GetSender()->m_activeCollection->GetDriverGoal();
				}
				else if(args->GetSender()->Holder == ControlItem::JoystickHolder::OperatorController)
				{
					Selected = args->GetSender()->m_activeCollection->GetOperatorGoal();
				}
				else
				{
					Selected = args->GetSender()->m_activeCollection->GetRobotGoal();
				}

				for(int i = 0; i < ((GoalButtonControl*)(args->GetSender()))->RemoveKeys.size(); i++)
				{
					Selected->RemoveGoal(((GoalButtonControl*)(args->GetSender()))->RemoveKeys.at(i));
				}
				TeleOpGoal goal = ((GoalButtonControl*)(args->GetSender()))->m_goal;
				SmartDashboard::PutString("GoalButtonControl Status", "Control Found");
				Goal* goalToAdd = SelectTeleOpGoal(args->GetSender()->m_activeCollection, goal, args->GetValue());
				Goal* OverGoal = new Goal_ControllerOverride(args->GetSender()->m_activeCollection, 1);
				OverGoal->IdentityKey = ((GoalButtonControl*)(args->GetSender()))->IdKey;
				goalToAdd->IdentityKey = ((GoalButtonControl*)(args->GetSender()))->IdKey;
				Selected->AddGoal(goalToAdd);
				goalToAdd->Activate();
				OverGoal->Activate();
				SmartDashboard::PutString("GoalButtonControl Status", "GoalActivated");

				/*
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
				SmartDashboard::PutString("GoalButtonControl Status", "GoalActivated");*/
				return;
			}
			args->GetSender()->SetToComponents(args->GetValue());
			SmartDashboard::PutNumber(args->GetSender()->name, args->GetValue());
		}
		else if (e->Type == 1)
		{
			auto SwerveArgs = (IEventArgs<double, double, double, ControlItem*>*)e;
			Log::General(" EVENT "+SwerveArgs->GetSender()->name);

			SwerveControl *DT = (SwerveControl*)(SwerveArgs->GetSender());
			DT->GetManager()->Set(SwerveArgs->GetVValue(), SwerveArgs->GetHValue(), SwerveArgs->GetSValue());

			SmartDashboard::PutNumber(SwerveArgs->GetSender()->name, SwerveArgs->GetVValue());
		}
		else if (e->Type == 2)
		{
			auto args = (LEventArgs<vector<double>, vector<string>, ControlItem*>*)e;
			Log::General(" EVENT "+args->GetSender()->name);

			//If the sender is a GoalButtonControl
			if (dynamic_cast<GoalAxisControl*>(args->GetSender()))
			{

				MultitaskGoal* Selected = nullptr;

				if(args->GetSender()->Holder == ControlItem::JoystickHolder::DriverController)
				{
					Selected = args->GetSender()->m_activeCollection->GetDriverGoal();
				}
				else if(args->GetSender()->Holder == ControlItem::JoystickHolder::OperatorController)
				{
					Selected = args->GetSender()->m_activeCollection->GetOperatorGoal();
				}
				else
				{
					Selected = args->GetSender()->m_activeCollection->GetRobotGoal();
				}
				Goal* ControlGoal = Selected->GetGoal(((GoalAxisControl*)(args->GetSender()))->IdKey);
				if(ControlGoal == nullptr)
				{
					for(int i = 0; i < ((GoalAxisControl*)(args->GetSender()))->RemoveKeys.size(); i++)
					{
						Selected->RemoveGoal(((GoalAxisControl*)(args->GetSender()))->RemoveKeys.at(i));
					}
					TeleOpGoal goal = ((GoalAxisControl*)(args->GetSender()))->m_goal;
					SmartDashboard::PutString("GoalAxisControl Status", "Control Found");
					Goal* goalToAdd = SelectTeleOpGoal(args->GetSender()->m_activeCollection, goal);
					//Goal* OverGoal = new Goal_ControllerOverride(args->GetSender()->m_activeCollection, 1);
					//OverGoal->IdentityKey = ((GoalButtonControl*)(args->GetSender()))->IdKey;
					goalToAdd->IdentityKey = ((GoalAxisControl*)(args->GetSender()))->IdKey;
					goalToAdd->CopyStringFrom(args->GetStrings(), 0);
					goalToAdd->CopyFrom(args->GetValues(), ((GoalAxisControl*)(args->GetSender()))->StartIndex);
					Selected->AddGoal(goalToAdd);
					goalToAdd->Activate();
					//OverGoal->Activate();
				}
				else
				{
					ControlGoal->CopyStringFrom(args->GetStrings(), 0);
					ControlGoal->CopyFrom(args->GetValues(), ((GoalAxisControl*)(args->GetSender()))->StartIndex);
					if(ControlGoal->GetStatus() == Goal::Goal_Status::eCompleted && ((GoalAxisControl*)(args->GetSender()))->RepeatWhenFinished)
						ControlGoal->Activate();
				}
				SmartDashboard::PutString("GoalAxisControl Status", "GoalActivated");
				return;
			}
			args->GetSender()->SetToComponents(args->GetValues().at(0));
			SmartDashboard::PutNumber(args->GetSender()->name, args->GetValues().at(0));
		}
		else
		{
			Log::Error("FAILED EVENT");
		}
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
	Holder = joy->GetPort() >= 0 && joy->GetPort() <= 1 ? (joy->GetPort() == 0 ? JoystickHolder::DriverController : JoystickHolder::OperatorController) : JoystickHolder::Other;
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

