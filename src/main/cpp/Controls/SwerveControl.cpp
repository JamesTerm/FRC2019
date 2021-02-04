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

#define _USE_MATH_DEFINES

#include "SwerveControl.h"
#include <cmath>

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

    m_Collection = ac;
}

double SwerveControl::Update(double _dTime)
{
    SwerveDrive->SetDelta(_dTime);
    SwerveDrive->SetL(Length);
    SwerveDrive->SetW(Width);
    SwerveDrive->UpdateModules();
    SwerveDrive->UpdateLoc();

    double rawH = -CalculateDeadZone((*joy).GetRawAxis(HAxis), DeadZone) * (Mult);
    double rawV = CalculateDeadZone((*joy).GetRawAxis(VAxis), DeadZone) * (Reversed ? -Mult : Mult);
    double rawS = -CalculateDeadZone((*joy).GetRawAxis(SAxis), DeadZone) * (Mult);
    rawV *= -1;

    if (Cal == SwerveControl::DriveCalculation::Field_Oriented)
    {
        double gyro = m_Collection->GetNavX()->GetConstAngle() * M_PI / 180;

        double temp = rawH * cos(gyro) + rawV * sin(gyro);
        rawV = -rawH * sin(gyro) + rawV * cos(gyro);
        rawH = temp;
    }
    else if (Cal == SwerveControl::DriveCalculation::Warthog)
    {
        rawS = rawH;
        rawH = 0;
    }

    ValueChanged(new IEventArgs<double, double, double, SwerveControl*>(rawV, rawH, rawS, this));

    return (rawH + rawV + rawS) / 3;
}

void SwerveControl::DeleteComponent()
{
    delete this;
}

SwerveControl::~SwerveControl() {}