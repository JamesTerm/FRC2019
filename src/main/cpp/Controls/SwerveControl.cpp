/****************************** Header ******************************\
Class Name:	SwerveControl
File Name:	SwerveControl.cpp
Summary:	Interface for swerve control.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Ian Poll
Email:	irobot9803@gmail.com
\*********************************************************************/

#include "SwerveControl.h"

using namespace std;
using namespace Controls;
using namespace Components;

SwerveControl::SwerveControl(Joystick *_joy, DriveCalculation _Cal, string _name, int _axisV, int _axisH, int _axisS, double _deadZone, bool _reversed, double _powerMultiplier, ActiveCollection* ac, SwerveManager *Manager, double _Length, double _Width) : ControlItem(_joy, _name, _reversed, _powerMultiplier, ac)
{
    SwerveDrive = Manager;
    Cal = _Cal;
    HAxis = _axisH;
    VAxis = _axisV;
    SAxis = _axisS;

    Length = _Length;
    Width = _Width;

    Mult = _powerMultiplier;
    DeadZone = _deadZone;
    Reversed = _reversed;
}

double SwerveControl::Update(double _dTime)
{
    SwerveDrive->SetDelta(_dTime);
    SwerveDrive->SetL(Length);
    SwerveDrive->SetW(Width);

    double rawH = CalculateDeadZone((*joy).GetRawAxis(HAxis), DeadZone) * (Mult);
    double rawV = CalculateDeadZone((*joy).GetRawAxis(VAxis), DeadZone) * (Reversed ? -Mult : Mult);
    double rawS = CalculateDeadZone((*joy).GetRawAxis(SAxis), DeadZone) * (Mult);

    ValueChanged(new IEventArgs<double, double, double, SwerveControl*>(rawV, rawH, rawS, this));

    return (rawH + rawV + rawS) / 3;
}

void SwerveControl::DeleteComponent()
{
    delete this;
}

SwerveControl::~SwerveControl() {}