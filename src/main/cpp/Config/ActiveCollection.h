/****************************** Header ******************************\
Class Name: ActiveCollection
File Name:	ActiveCollection.h
Summary: Stores all Components on the robot controlled by the software.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson, Chris Weeks
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com, chrisrweeks@aol.com
\********************************************************************/

#pragma once

#ifndef SRC_CONFIG_ACTIVECOLLECTION_H_
#define SRC_CONFIG_ACTIVECOLLECTION_H_

#include <vector>
#include <utility>

#include "../Components/VictorSPItem.h"
#include "../Components/VictorSPXItem.h"
#include "../Components/TalonSRXItem.h"
#include "../Components/EncoderItem.h"
#include "../Components/PotentiometerItem.h"
#include "../Components/DoubleSolenoidItem.h"
#include "../Components/DigitalInputItem.h"
#include "../Components/NavX.h"
#include "../Components/SparkMaxItem.h"
#include "../Components/PDBManager.h"


class MultitaskGoal;

using namespace std;
using namespace Components;

namespace Configuration
{
	class ActiveCollection
	{
		public:
			enum ComponentType
			{
				VictorSP
			};

			ActiveCollection();
			virtual ~ActiveCollection(){}

			NativeComponent* Get(string name);
			VictorSPItem* GetVictor(string name);
			TalonSRXItem* GetTalon(string name);
			EncoderItem* GetEncoder(string name);
			void DoubleSolenoidDefault();
			NavX* GetNavX();
			int GetSize();
			vector<NativeComponent*> GetRawComponent();
			void Add(NativeComponent *component);
			void AddEvent(Event *event);
			vector<Event*> EventMap;

			void DeleteAll();

			void SetActiveGoal(MultitaskGoal* g) {activeGoal = g;}
			MultitaskGoal* GetActiveGoal() {return activeGoal;}

			void SetOverdrive(bool o) {overdrive = o;}
			bool GetOverdrive() {return overdrive;}

			void SetAuto(string Auto) {AutoSele = Auto;}
			void SetAutoOverride(bool Override) {AutoOverride = Override;}
			string GetAuto() {return AutoSele;}
			bool ConfigOverride() {return AutoOverride;}

			void SetPDP(double TimeOut, double CurrentThres, double Lower, bool run) {PDP = new PDBManager(TimeOut, CurrentThres, Lower, run);}
			PDBManager* GetPDBManager() {return PDP;}
			
	private:
		vector<NativeComponent*> activeCollection;
		MultitaskGoal* activeGoal = nullptr;
		bool overdrive;
		bool AutoOverride = false;
		string AutoSele;
		PDBManager* PDP = nullptr;
	};
}

#endif /* SRC_CONFIG_ACTIVECOLLECTION_H_ */
