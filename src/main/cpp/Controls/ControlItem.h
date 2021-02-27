/****************************** Header ******************************\
Class Name:	ControlItem
File Name:	ControlItem.h
Summary:	Abstraction for managing all driver and operator controls
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson
Email:	dylantrwatson@gmail.com
\*********************************************************************/

#pragma once

#include <frc/WPILib.h>
#include <vector>

#include "../Util/Constants.h" 
#include "../Components/OutputComponent.h"
#include "../Global.h"
#include "../Config/ActiveCollection.h"
#include "../Util/SmartDashboard.h"
#include "../Components/Swerve/SwerveManager.h"

using namespace frc;
using namespace std;
using namespace Util;
using namespace Components;
using namespace Configuration;

namespace Controls
{
	class ControlItem 
	{
		protected:
			bool reversed;
			double powerMultiplier;
			vector<OutputComponent*> components;

		public:

			enum JoystickHolder
			{
				DriverController = 0,
				OperatorController = 1,
				Other = 2
			};

			JoystickHolder Holder;

			ActiveCollection* m_activeCollection;
			Joystick *joy;
			ControlItem();
			ControlItem(Joystick *_joy, string _name, bool _reversed, double _powerMultiplier, ActiveCollection* activeCollection);
			virtual double Update(double _dTime) = 0;
			void AddComponent(OutputComponent *component);
			vector<string> GetComponents();
			string name;
			Event ValueChanged;
			void SetToComponents(double val);
			virtual void DeleteComponent();
			virtual ~ControlItem();
	};
}