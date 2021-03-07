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
#include "../Components/ServoItem.h"


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
			Motor* GetMotor(string name);
			VictorSPItem* GetVictor(string name);
			TalonSRXItem* GetTalon(string name);
			EncoderItem* GetEncoder(string name);
			ServoItem* GetServo(string name);
			void DoubleSolenoidDefault();
			NavX* GetNavX();
			int GetSize();
			vector<NativeComponent*> GetRawComponent();
			void Add(NativeComponent *component);
			void AddEvent(Event *event);
			vector<Event*> EventMap;

			void DeleteAll();

			void ResetSuperior_Goal();

			void ProcessSuperior_Goal(double dTime);

			void SetDriverGoal(MultitaskGoal* g) {Driver_Goal = g;}
			MultitaskGoal* GetDriverGoal() {return Driver_Goal;}

			void SetOperatorGoal(MultitaskGoal* g) {Operator_Goal = g;}
			MultitaskGoal* GetOperatorGoal() {return Operator_Goal;}

			void SetRobotGoal(MultitaskGoal* g) {Robot_Goal = g;}
			MultitaskGoal* GetRobotGoal() {return Robot_Goal;}

			void SetLoopTime(double Time){WaitTime = Time;}
			double GetWaitTime() {return WaitTime;}

			int CreateAndAddProfile(string Name, double P, double I, double D, double MaxChange = 0.1, double Bias = -1, double InnerMin = 0, double InnerMax = 0, double Min = -1, double Max = 1, double Thres = 0.01, int index = -1);
			int AddProfile(ProfileData* Data, int index = -1);
			ProfileData* GetProfile(int i);
			ProfileData* GetProfile(string Name);

			void SetOverdrive(bool o) {overdrive = o;}
			bool GetOverdrive() {return overdrive;}

			void SetAuto(string Auto) {AutoSele = Auto;}
			void SetAutoOverride(bool Override) {AutoOverride = Override;}
			string GetAuto() {return AutoSele;}
			bool ConfigOverride() {return AutoOverride;}
			void SetAutoScale(double Scale) {AutoScale = Scale;}
			double GetAutoScale() {return AutoScale;}

			void SetIsSwerveDrive(bool Yes) {SwerveDrive = Yes;}
			bool IsSwerveDrive() {return SwerveDrive;}

			void SetPDP(double TimeOut, double CurrentThres, double Lower, bool run) {PDP = new PDBManager(TimeOut, CurrentThres, Lower, run);}
			PDBManager* GetPDBManager() {return PDP;}

			void UpdateComponents();
			
	private:
		vector<NativeComponent*> activeCollection;
		MultitaskGoal* superior_Goal = nullptr;

		MultitaskGoal* Driver_Goal = nullptr;
		MultitaskGoal* Operator_Goal = nullptr;
		MultitaskGoal* Robot_Goal = nullptr;

		vector<ProfileData*> Profiles;
		ProfileData* DefaultData = new ProfileData();
		
		bool overdrive;
		bool AutoOverride = false;
		string AutoSele = "????";
		double AutoScale = 1;
		bool SwerveDrive = false;

		PDBManager* PDP = nullptr;

		double WaitTime = 0.01;
	};
}

#endif /* SRC_CONFIG_ACTIVECOLLECTION_H_ */
