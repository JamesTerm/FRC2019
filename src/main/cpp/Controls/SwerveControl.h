/****************************** Header ******************************\
Class Name:	SwerveControl
File Name:	SwerveControl.h
Summary:	Interface for swerve control.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Ian Poll
Email:	irobot9803@gmail.com
\*********************************************************************/


#ifndef SRC_CONTROLS_SWERVECONTROL_H_
#define SRC_CONTROLS_SWERVECONTROL_H_

#include <frc/WPILib.h>
#include "ControlItem.h"

using namespace frc;
using namespace std;
using namespace Configuration;

namespace Controls
{
	class SwerveControl : public ControlItem
	{
        
		public:
            enum DriveCalculation {Robot_Oriented = 0, Field_Oriented = 1};

			SwerveControl(Joystick *_joy, DriveCalculation _Cal, string _name, int _axisV, int _axisH, int _axisS, double _deadZone, bool _reversed, double _powerMultiplier, ActiveCollection* ac, SwerveManager *Manager, double _Length, double _Width);
			virtual double Update(double _dTime) override;
			virtual ~SwerveControl();

            SwerveManager *GetManager(){return SwerveDrive;};

		private:
			SwerveManager *SwerveDrive;
            DriveCalculation Cal;

            double CalculateDeadZone(double Axis, double dz)
            {
                if (abs(Axis) > dz)
                    return ((1 / (1 - dz)) * (Axis - dz));
                return 0;
            };

            int HAxis;
            int VAxis;
            int SAxis;

            double Length;
            double Width;

            double Mult;
            double DeadZone = 0;
            bool Reversed;
	};
}

#endif /* SRC_CONTROLS_AXISCONTROL_H_ */
