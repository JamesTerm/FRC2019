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

using namespace frc;
using namespace std;
using namespace Util;
using namespace Components;
using namespace Configuration;

namespace Controls
{
	class ControlItem 
	{
		private:
			ActiveCollection* m_activeCollection;

		protected:
			bool reversed;
			double powerMultiplier;
			vector<OutputComponent*> components;

		public:
			Joystick *joy;
			ControlItem();
			ControlItem(Joystick *_joy, string _name, bool _reversed, double _powerMultiplier, ActiveCollection* activeCollection);
			virtual double Update() = 0;
			void AddComponent(OutputComponent *component);
			vector<string> GetComponents();
			string name;
			Event ValueChanged;
			void SetToComponents(double val);
			virtual ~ControlItem();
	};
}