/****************************** Header ******************************\
Class Name: SwerveManager
File Name:	SwerveManager.cpp
Summary: Manager that holds the Swerve modules
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#define _USE_MATH_DEFINES

#include "SwerveManager.h"
#include <cmath>

using namespace std;
using namespace frc;
using namespace Components;

SwerveManager::SwerveManager(string name, SwerveModule *FrontLeft, SwerveModule *FrontRight, SwerveModule *BackLeft, SwerveModule *BackRight) : OutputComponent(name)
{
    FL = FrontLeft;
    FR = FrontRight;
    BL = BackLeft;
    BR = BackRight;
}

void SwerveManager::DeleteComponent()
{
    delete this;
}

void SwerveManager::Set(double val)
{
    FL->Set(val);
    FR->Set(val);
    BL->Set(val);
    BR->Set(val);
}

void SwerveManager::SetSwivel(double val)
{
    FL->SetSwivel(val);
    FR->SetSwivel(val);
    BL->SetSwivel(val);
    BR->SetSwivel(val);
}

void SwerveManager::Set(double rawV, double rawH, double rawS)
{
    double piVal = M_PI;

    double r = sqrt((Length * Length) + (Width * Width));
    rawV *= -1;

    double a = rawH - rawS * (Length / r);
    double b = rawH + rawS * (Length / r);
    double c = rawV - rawS * (Width / r);
    double d = rawV + rawS * (Width / r);

    double backRightSpeed = sqrt ((a * a) + (d * d));
    double backLeftSpeed = sqrt ((a * a) + (c * c));
    double frontRightSpeed = sqrt ((b * b) + (d * d));
    double frontLeftSpeed = sqrt ((b * b) + (c * c));

    double backRightAngle = (atan2 (a, d) / piVal) * 180;
    double backLeftAngle = (atan2 (a, c) / piVal) * 180;
    double frontRightAngle = (atan2 (b, d) / piVal) * 180;
    double frontLeftAngle = (atan2 (b, c) / piVal) * 180;

    if (FL->SetTargetSwivel(frontLeftAngle) &&
        FR->SetTargetSwivel(frontRightAngle) &&
        BL->SetTargetSwivel(backLeftAngle) &&
        BR->SetTargetSwivel(backRightAngle))
    {
        FL->Set(frontLeftSpeed);
        FR->Set(frontRightSpeed);
        BL->Set(backLeftSpeed);
        BR->Set(backRightSpeed);
    }
    else
    {
        FL->Set(frontLeftSpeed / 5);
        FR->Set(frontRightSpeed / 5);
        BL->Set(backLeftSpeed / 5);
        BR->Set(backRightSpeed / 5);
        Log::General("Waiting for swivel motors to get their shit together");
    }
}

bool SwerveManager::SetSwivelTarget(double Target)
{
    return FL->SetTargetSwivel(Target) && FR->SetTargetSwivel(Target) && BL->SetTargetSwivel(Target) && BR->SetTargetSwivel(Target);
}

bool SwerveManager::SetWheelTarget(double Target)
{
    return FL->SetTargetWheel(Target) && FR->SetTargetWheel(Target) && BL->SetTargetWheel(Target) && BR->SetTargetWheel(Target);
}

bool SwerveManager::SetTarget(double Wheel_Target, double Swivel_Target)
{
    return SwerveManager::SetSwivelTarget(Swivel_Target) && SwerveManager::SetWheelTarget(Wheel_Target);
}

void SwerveManager::SetDelta(double D_Time)
{
    FL->SetDeltaTime(D_Time);
    FR->SetDeltaTime(D_Time);
    BL->SetDeltaTime(D_Time);
    BR->SetDeltaTime(D_Time);
}

void SwerveManager::ResetPID()
{
    FL->ResetPID();
    FR->ResetPID();
    BL->ResetPID();
    BR->ResetPID();
}

void SwerveManager::ResetSwivelEnc()
{
    FL->ResetSwivelEnc();
    FR->ResetSwivelEnc();
    BL->ResetSwivelEnc();
    BR->ResetSwivelEnc();
}

void SwerveManager::ResetWheelEnc()
{
    FL->ResetWheelEnc();
    FR->ResetWheelEnc();
    BL->ResetWheelEnc();
    BR->ResetWheelEnc();
}

double SwerveManager::Get()
{
    return (FL->Get() + FR->Get() + BL->Get() + BR->Get()) / 4;
}

void SwerveManager::DefaultSet()
{
    Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void SwerveManager::Set(DoubleSolenoid::Value value)
{
    Log::Error("WHY DID YOU CALL THE DOUBLESOLENOID SET FOR A MOTOR?!? Yell at your programmers!");
}