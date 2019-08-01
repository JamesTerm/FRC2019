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

#include "ActiveCollection.h"
#include "../Goals/Goal.h"

using namespace std;
using namespace Configuration;

/**
 * Default Constructor
**/
ActiveCollection::ActiveCollection()
{
	activeGoal = new MultitaskGoal(this, false);
	overdrive = false;
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
		Log::Error("Cannot find component " + name + ", it does not exist in the active collection!\nCheck your type correlation!");
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
 * Method to return a Solenoid of a certain name
 **/
DoubleSolenoidItem* ActiveCollection::GetDoubleSolenoid(string name)
{
	DoubleSolenoidItem *ret = nullptr;
	try
	{
		for(int i=0; i<(int)activeCollection.size();i++){
			if((*activeCollection[i]).name == (string)name){
				ret=(DoubleSolenoidItem*)activeCollection[i];
			}
		}
		if (!ret) throw "Gabe youre dum";
	}
	catch(...){
		Log::Error("Cannot find DoubleSolenoid " + name + ", it does not exist in the active collection!");
	}
	return ret;
	

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