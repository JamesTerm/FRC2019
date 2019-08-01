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
			DoubleSolenoidItem* GetDoubleSolenoidDefault();
			NavX* GetNavX();
			int GetSize();
			vector<NativeComponent*> GetRawComponent();
			void Add(NativeComponent *component);
			void AddEvent(Event *event);
			vector<Event*> EventMap;

			void SetActiveGoal(MultitaskGoal* g) {activeGoal = g;}
			MultitaskGoal* GetActiveGoal() {return activeGoal;}

			void SetOverdrive(bool o) {overdrive = o;}
			bool GetOverdrive() {return overdrive;}
			
	private:
		vector<NativeComponent*> activeCollection;
		MultitaskGoal* activeGoal;
		bool overdrive;
	};
}

#endif /* SRC_CONFIG_ACTIVECOLLECTION_H_ */
