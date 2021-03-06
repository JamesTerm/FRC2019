/****************************** Header ******************************\
Class Name: ActiveCollection
File Name:	ActiveCollection.cpp
Summary: Stores all Components on the robot controlled by the software.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson, Chris Weeks
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com, chrisrweeks@aol.com
\********************************************************************/

#include <iostream>
#include <typeinfo>
#include <typeindex>

#include "ActiveCollection.h"
#include "../Goals/Goal.h"

using namespace std;
using namespace Configuration;

/**
 * Default Constructor
**/
ActiveCollection::ActiveCollection()
{
	superior_Goal = new MultitaskGoal(this, true);

	Driver_Goal = new MultitaskGoal(this, true);
	Operator_Goal = new MultitaskGoal(this, true);
	Robot_Goal = new MultitaskGoal(this, true);
	
	superior_Goal->AddGoal(Driver_Goal);
	superior_Goal->AddGoal(Operator_Goal);
	superior_Goal->AddGoal(Robot_Goal);

	overdrive = false;
}

void ActiveCollection::ResetSuperior_Goal()
{
	superior_Goal->~MultitaskGoal();

	superior_Goal = new MultitaskGoal(this, true);

	Driver_Goal = new MultitaskGoal(this, true);
	Operator_Goal = new MultitaskGoal(this, true);
	Robot_Goal = new MultitaskGoal(this, true);
	
	superior_Goal->AddGoal(Driver_Goal);
	superior_Goal->AddGoal(Operator_Goal);
	superior_Goal->AddGoal(Robot_Goal);

	superior_Goal->Activate();
}

void ActiveCollection::ProcessSuperior_Goal(double dTime)
{
	superior_Goal->Process(dTime);
}

/**
 * Method to return a NativeComponent of a certain type and name
**/

NativeComponent* ActiveCollection::Get(string name)
{
	NativeComponent *ret = nullptr;
	try{
		for(int i=0; i<(int)activeCollection.size();i++)
			if((*activeCollection[i]).name == (string)name)
				ret=activeCollection[i];
		if(!ret) throw "AHHH";
	}
	catch(...){
		Log::Error("Cannot find component " + name + ", it does not exist in the active collection!");
	}
	return ret;
}

/**
 * Method to return a Motor of a certain name
**/

Motor* ActiveCollection::GetMotor(string name)
{
	Motor *ret = nullptr;
	try{
		for(int i=0; i<(int)activeCollection.size();i++){
			if((*activeCollection[i]).name == (string)name){
				ret=(Motor*)activeCollection[i];
			}
		}
		if (!ret) throw "AHHH";
	}
	catch(...){
		Log::Error("Cannot find Motor " + name + ", it does not exist in the active collection!");
	}
	return ret;
}

/**
 * Method to return a VictorSP of a certain name
**/
VictorSPItem* ActiveCollection::GetVictor(string name)
{
	VictorSPItem *ret = nullptr;
	try{
		for(int i=0; i<(int)activeCollection.size();i++){
			if((*activeCollection[i]).name == (string)name){
				ret=(VictorSPItem*)activeCollection[i];
			}
		}
		if (!ret) throw "AHHH";
	}
	catch(...){
		Log::Error("Cannot find victor " + name + ", it does not exist in the active collection!");
	}
	return ret;
}

/**
 * Method to return a TalonSRX of a certain name
**/
TalonSRXItem* ActiveCollection::GetTalon(string name)
{
	TalonSRXItem *ret = nullptr;
	try{
		for(int i=0; i<(int)activeCollection.size();i++){
			if((*activeCollection[i]).name == (string)name){
				ret=(TalonSRXItem*)activeCollection[i];
			}
		}
		if (!ret) throw "AHHH";
	}
	catch(...){
		Log::Error("Cannot find talon " + name + ", it does not exist in the active collection!");
	}
	return ret;
}

/**
 * Method to return all Solenoids
 **/
void ActiveCollection::DoubleSolenoidDefault()
{
	try
	{
		for(int i=0; i<(int)activeCollection.size();i++)
		{
			if(dynamic_cast<DoubleSolenoidItem*>(activeCollection[i]))
				((DoubleSolenoidItem*)activeCollection[i])->DefaultSet();
		}
	}
	catch(...){
		Log::Error("Cannot find DoubleSolenoids, it does not exist in the active collection!");
	}
}

/**
 * Method to return an Encoder of a certain name
**/
EncoderItem* ActiveCollection::GetEncoder(string name)
{
	EncoderItem *ret = nullptr;
	try{
		for(int i=0; i<(int)activeCollection.size();i++){
			if((*activeCollection[i]).name == (string)name){
				ret=(EncoderItem*)activeCollection[i];
			}
		}
		if (!ret) throw "AHHH";
	}
	catch(...){
		Log::Error("Cannot find encoder " + name + ", it does not exist in the active collection!");
	}
	return ret;
}

/**
 * Method to return a Servo of a certain name
**/
ServoItem* ActiveCollection::GetServo(string name)
{
	ServoItem *ret = nullptr;
	try{
		for(int i=0; i<(int)activeCollection.size();i++){
			if((*activeCollection[i]).name == (string)name){
				ret=(ServoItem*)activeCollection[i];
			}
		}
		if (!ret) throw "AHHH";
	}
	catch(...){
		Log::Error("Cannot find servo " + name + ", it does not exist in the active collection!");
	}
	return ret;
}

/**
 * Method to return the NavX
**/
NavX* ActiveCollection::GetNavX()
{
	NavX *ret = nullptr;
	try{
		for(int i=0; i<(int)activeCollection.size();i++){
			if((*activeCollection[i]).name == "NavX"){
				ret=(NavX*)activeCollection[i];
			}
		}
		if (!ret) throw "AHHH";
	}
	catch(...){
		Log::Error("Cannot find the NavX, check the config.");
	}
	return ret;
}

/**
 * Method to return the current size of the ActiveCollection list
**/
int ActiveCollection::GetSize(){
	return (int)activeCollection.size();
}

/**
 * Adds a component to the ActiveCollection
**/
void ActiveCollection::Add(NativeComponent *component){
	try{
		activeCollection.push_back(component);
	}
	catch(...){
		Log::Error("Cannot add component " + (*component).name);
	}
}

vector<NativeComponent*> ActiveCollection::GetRawComponent(){
	return activeCollection;
}

void ActiveCollection::AddEvent(Event *event){
	EventMap.push_back(event);
}

void ActiveCollection::DeleteAll()
{
	for(int i = 0; i < activeCollection.size(); i++)
	{
		activeCollection[i]->DeleteComponent();
	}
	activeCollection.clear();
	for(int i = 0; i < Profiles.size(); i++)
	{
		delete Profiles[i];
	}
	Profiles.clear();
}

void ActiveCollection::UpdateComponents()
{
	for(int i = 0; i < activeCollection.size(); i++)
	{
		activeCollection[i]->UpdateComponent();
	}
}

int ActiveCollection::CreateAndAddProfile(string Name, double P, double I, double D, double MaxChange, double Bias, double Min, double Max, int index)
{
	ProfileData* Data = new ProfileData(P, I, D, MaxChange, (Bias <= 0 ? P * 100 : Bias), Min, Max, Name);
	return ActiveCollection::AddProfile(Data, index);
}

int ActiveCollection::AddProfile(ProfileData* Data, int index)
{
	if(index < 0)
	{
		Profiles.push_back(Data);
		return Profiles.size() - 1;
	}
	else
	{
		if(index >= Profiles.size())
		{
			int sizeDist = (index - Profiles.size()) + 1;
			for(int i = 0; i < sizeDist; i++)
			{
				ProfileData* NewData = new ProfileData();
				Profiles.push_back(NewData);
			}
		}
		delete Profiles[index];
		Profiles[index] = Data;
		return index;
	}
}

ProfileData* ActiveCollection::GetProfile(int i)
{
	if(i >= Profiles.size())
	{
		Log::Error("Cannot find Profile with index: " + to_string(i) + "----Returning Default");
		return DefaultData;
	}
	else
	{
		return Profiles[i];
	}
}

ProfileData* ActiveCollection::GetProfile(string Name)
{
	for(int i = 0; i < Profiles.size(); i++)
	{
		if(Profiles[i]->Name.compare(Name) == 0)
		{
			return Profiles[i];
		}
	}
	Log::Error("Cannot find Profile with Name: " + Name + "----Returning Default");
	return DefaultData;
}