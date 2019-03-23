/****************************** Header ******************************\
Class Name:	AxisControl inherits ControlItem
File Name:	AxisControl.h
Summary:	Interface for an axis control.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Dylan Watson
Email:	dylantrwatson@gmail.com
\*********************************************************************/


#ifndef SRC_CONTROLS_AXISCONTROL_H_
#define SRC_CONTROLS_AXISCONTROL_H_

#include <frc/WPILib.h>
#include "ControlItem.h"
#include "../Components/PotentiometerItem.h"
#include "../Goals/FRC2019_Goals.h"

using namespace frc;
using namespace std;
using namespace Configuration;

namespace Controls
{
	class AxisControl : public ControlItem
	{
		private:
			int axis;
			double deadZone;
			int getSign(double val);
			double currentPow;
			double previousPow;
			int channel;
			double gane;
			bool isLift;
			ActiveCollection *m_activeCollection;
			//TODO: downVal
			bool isIdle = false;
			double targetVal;

		public:
			AxisControl();
			AxisControl(Joystick *_joy, string _name, int _axis, double _deadZone, bool _reversed, double _powerMultiplier, ActiveCollection* ac);
			virtual double Update(double _dTime) override;
			void SetLift(double _gane, ActiveCollection *activeCollection);
			virtual ~AxisControl();
	};
}

#endif /* SRC_CONTROLS_AXISCONTROL_H_ */
