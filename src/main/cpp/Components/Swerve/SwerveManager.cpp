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

SwerveManager::SwerveManager(string name, bool Wait, SwerveModule *FrontLeft, SwerveModule *FrontRight, SwerveModule *BackLeft, SwerveModule *BackRight, double _Length, double _Width) : OutputComponent(name)
{
    Modules.push_back(FrontLeft);
    Modules.push_back(BackLeft);
    Modules.push_back(FrontRight);
    Modules.push_back(BackRight);
    SwerveManager::SetL(_Length);
    SwerveManager::SetW(_Width);
    Pos = new double_Vector2();
    WaitSwivel = Wait;
    SwerveManager::TableSetUp();
}

SwerveManager::SwerveManager(string name, bool Wait, SwerveModule *FrontLeft, SwerveModule *FrontRight, SwerveModule *BackLeft, SwerveModule *BackRight, NavX *Nav, double _Length, double _Width) : OutputComponent(name)
{
    Modules.push_back(FrontLeft);
    Modules.push_back(BackLeft);
    Modules.push_back(FrontRight);
    Modules.push_back(BackRight);
    SwerveManager::SetL(_Length);
    SwerveManager::SetW(_Width);
    Pos = new double_Vector2();
    RobotNav = Nav;
    WaitSwivel = Wait;
    SwerveManager::TableSetUp();
}

SwerveManager::SwerveManager(string name, bool Wait, vector<SwerveModule*> Swerve_Modules, NavX *Nav, double _Length, double _Width) : OutputComponent(name)
{
    Modules = Swerve_Modules;
    Pos = new double_Vector2();
    SwerveManager::SetL(_Length);
    SwerveManager::SetW(_Width);
    RobotNav = Nav;
    WaitSwivel = Wait;
    SwerveManager::TableSetUp();
}

void SwerveManager::TableSetUp()
{
    OutputTable->PutNumber("1A-Wheel_Size", 1.35);
    OutputTable->PutNumber("2A-Scale", 1);
    OutputTable->PutNumber("BotX", 0);
    OutputTable->PutNumber("BotY", 0);
    OutputTable->PutNumber("BotZ", 0);
    SimPos = new double_Vector2();
}

void SwerveManager::DeleteComponent()
{
    if(SimPos != nullptr)
        delete SimPos;
    delete Pos;
    delete this;
}

void SwerveManager::Set(double val)
{
    for(int i = 0; i < Modules.size(); i++)
    {
        Modules.at(i)->Set(val);
    }
}

void SwerveManager::SetSwivel(double val)
{
    for(int i = 0; i < Modules.size(); i++)
    {
        Modules.at(i)->SetSwivel(val);
    }
}

void SwerveManager::UpdateModules()
{
    for(int i = 0; i < Modules.size(); i++)
    {
        Modules.at(i)->UpdateWheelRate();
    }
}

void SwerveManager::UpdateLoc()
{
    if (RobotNav != nullptr)
    {
        SwerveManager::SetWheelDiameter(OutputTable->GetNumber("1A-Wheel_Size", 1));

        double FL_B = sin(SwerveManager::Get(SwerveModule::Location::Front_Left)->GetSwivelTarget() * M_PI / 180) * SwerveManager::Get(SwerveModule::Location::Front_Left)->GetWheelRate();
        double FR_B = sin(SwerveManager::Get(SwerveModule::Location::Front_Right)->GetSwivelTarget() * M_PI / 180) * SwerveManager::Get(SwerveModule::Location::Front_Right)->GetWheelRate();
        double BL_A = sin(SwerveManager::Get(SwerveModule::Location::Back_Left)->GetSwivelTarget() * M_PI / 180) * SwerveManager::Get(SwerveModule::Location::Back_Left)->GetWheelRate();
        double BR_A = sin(SwerveManager::Get(SwerveModule::Location::Back_Right)->GetSwivelTarget() * M_PI / 180) * SwerveManager::Get(SwerveModule::Location::Back_Right)->GetWheelRate();
    
        double FR_C = cos(SwerveManager::Get(SwerveModule::Location::Front_Right)->GetSwivelTarget() * M_PI / 180) * SwerveManager::Get(SwerveModule::Location::Front_Right)->GetWheelRate();
        double BR_C = cos(SwerveManager::Get(SwerveModule::Location::Front_Right)->GetSwivelTarget() * M_PI / 180) * SwerveManager::Get(SwerveModule::Location::Back_Right)->GetWheelRate();
        double FL_D = cos(SwerveManager::Get(SwerveModule::Location::Front_Left)->GetSwivelTarget() * M_PI / 180) * SwerveManager::Get(SwerveModule::Location::Front_Left)->GetWheelRate();
        double BL_D = cos(SwerveManager::Get(SwerveModule::Location::Back_Left)->GetSwivelTarget() * M_PI / 180) * SwerveManager::Get(SwerveModule::Location::Back_Left)->GetWheelRate();
    
        double A = (BR_A + BL_A) / 2;
        double B = (FL_B + FR_B) / 2;

        double C = (FR_C + BR_C) / 2;
        double D = (FL_D + BL_D) / 2;

        double R1 = (B - A) / Length;
        double R2 = (C - D) / Width;
        double R = (R1 + R2) / 2;

        double F1 = R * (Length / 2) + A;
        double F2 = -R * (Length / 2) + B;
        double F = (F1 + F2) / 2;

        double L1 = R * (Width / 2) + C;
        double L2 = -R * (Width / 2) + D;
        double L = (L1 + L2) / 2;

        double gyro = -RobotNav->GetConstAngle() * M_PI / 180;

        double temp = F * cos(gyro) + L * sin(gyro);
        L = -F * sin(gyro) + L * cos(gyro);
        F = temp;

        /*Pos->Y += L * Del_Time;
        Pos->X -= F * Del_Time;*/

        /*double ChangeX = 0;
        double ChangeY = 0;
        if(abs(L) != 0 && abs(F) != 0)
        {
            ChangeX = Pos->LastX - F;
            ChangeY = Pos->LastY - L;
            Pos->LastX = F;
            Pos->LastY = L;
        }*/
        Pos->Y += L * OutputTable->GetNumber("2A-Scale", 1);
        Pos->X -= F * OutputTable->GetNumber("2A-Scale", 1);
    }
    OutputTable->PutNumber("Loc-X", Pos->X);
    OutputTable->PutNumber("Loc-Y", Pos->Y);
    SimPos->X = OutputTable->GetNumber("BotX", 0);
    SimPos->Y = OutputTable->GetNumber("BotZ", 0);
}

void SwerveManager::ResetLoc()
{
    Pos->X = 0;
    Pos->Y = 0;
}

void SwerveManager::Set(double rawV, double rawH, double rawS)
{
    double piVal = M_PI;

    double r = sqrt((Length * Length) + (Width * Width));

    double a = rawH - rawS * (Length / r);
    double b = rawH + rawS * (Length / r);
    double c = rawV - rawS * (Width / r);
    double d = rawV + rawS * (Width / r);

    double backRightSpeed = sqrt ((a * a) + (d * d));
    double backLeftSpeed = sqrt ((a * a) + (c * c));
    double frontRightSpeed = sqrt ((b * b) + (d * d));
    double frontLeftSpeed = sqrt ((b * b) + (c * c));

    double backRightAngle = (atan2 (a, d) * 180) / piVal;
    double backLeftAngle = (atan2 (a, c) * 180)  / piVal;
    double frontRightAngle = (atan2 (b, d) * 180)/ piVal;
    double frontLeftAngle = (atan2 (b, c) * 180) / piVal;

    double MaxVal = frontLeftSpeed;
    MaxVal = SwerveManager::GetMax(MaxVal, frontRightSpeed);
    MaxVal = SwerveManager::GetMax(MaxVal, backLeftSpeed);
    MaxVal = SwerveManager::GetMax(MaxVal, backRightSpeed);

    if (MaxVal > MaxValParam)
    {
        backRightSpeed /= MaxVal;
        backLeftSpeed /= MaxVal;
        frontRightSpeed /= MaxVal;
        frontLeftSpeed /= MaxVal;
    }

    bool SFL = SwerveManager::SetSwivelTargetAt(SwerveModule::Location::Front_Left, frontLeftAngle);
    bool SFR = SwerveManager::SetSwivelTargetAt(SwerveModule::Location::Front_Right, frontRightAngle);
    bool SBL = SwerveManager::SetSwivelTargetAt(SwerveModule::Location::Back_Left, backLeftAngle);
    bool SBR = SwerveManager::SetSwivelTargetAt(SwerveModule::Location::Back_Right, backRightAngle);

    if ((SFL &&
        SFR &&
        SBL &&
        SBR) || !WaitSwivel)
    {
        
        SwerveManager::Get(SwerveModule::Location::Front_Left)->GetWheelMtr()->SetPower(frontLeftSpeed, Del_Time);
        SwerveManager::Get(SwerveModule::Location::Front_Right)->GetWheelMtr()->SetPower(frontRightSpeed, Del_Time);
        SwerveManager::Get(SwerveModule::Location::Back_Left)->GetWheelMtr()->SetPower(backLeftSpeed, Del_Time);
        SwerveManager::Get(SwerveModule::Location::Back_Right)->GetWheelMtr()->SetPower(backRightSpeed, Del_Time);
        /*
        SwerveManager::SetWheelAt(SwerveModule::Location::Front_Left, frontLeftSpeed);
        SwerveManager::SetWheelAt(SwerveModule::Location::Front_Right, frontRightSpeed);
        SwerveManager::SetWheelAt(SwerveModule::Location::Back_Left, backLeftSpeed);
        SwerveManager::SetWheelAt(SwerveModule::Location::Back_Right, backRightSpeed);*/
    }
    else
    {
        SwerveManager::Set(0);
        Log::General("Waiting for swivel motors to get their shit together");
    }
}

void SwerveManager::UpdateSystem(double D_Time)
{
    SwerveManager::SetDelta(D_Time);
    SwerveManager::UpdateModules();
    SwerveManager::UpdateLoc();
}

bool SwerveManager::SetSwivelTargetAt(SwerveModule::Location Loc, double Target)
{
    bool Finished = true;
    for(int i = 0; i < Modules.size(); i++)
    {
        if (Modules.at(i)->GetLocation() == Loc)
            Finished = Finished && Modules.at(i)->SetTargetSwivel(Target);
    }
    return Finished;
}

bool SwerveManager::SetWheelTargetAt(SwerveModule::Location Loc, double Target)
{
    bool Finished = true;
    for(int i = 0; i < Modules.size(); i++)
    {
        if (Modules.at(i)->GetLocation() == Loc)
            Finished = Finished && Modules.at(i)->SetTargetWheel(Target);
    }
    return Finished;
}

void SwerveManager::SetSwivelAt(SwerveModule::Location Loc, double Power)
{
    for(int i = 0; i < Modules.size(); i++)
    {
        if (Modules.at(i)->GetLocation() == Loc)
            Modules.at(i)->SetSwivel(Power);
    }
}

void SwerveManager::SetWheelAt(SwerveModule::Location Loc, double Power)
{
    for(int i = 0; i < Modules.size(); i++)
    {
        if (Modules.at(i)->GetLocation() == Loc)
            Modules.at(i)->Set(Power);
    }
}

void SwerveManager::SetWheelDiameter(double Diameter)
{
    for(int i = 0; i < Modules.size(); i++)
    {
        Modules.at(i)->SetWheelSize(Diameter);
    }
}

bool SwerveManager::SetSwivelTarget(double Target)
{
    bool Finished = true;
    for(int i = 0; i < Modules.size(); i++)
    {
        Finished = Finished && Modules.at(i)->SetTargetSwivel(Target);
    }
    return Finished;
}

bool SwerveManager::SetWheelTarget(double Target)
{
    bool Finished = true;
    for(int i = 0; i < Modules.size(); i++)
    {
        Finished = Finished && Modules.at(i)->SetTargetWheel(Target);
    }
    return Finished;
}

bool SwerveManager::SetTarget(double Wheel_Target, double Swivel_Target)
{
    bool Can = SwerveManager::SetSwivelTarget(Swivel_Target);
    return  Can ? SwerveManager::SetWheelTarget(Wheel_Target) : false;
}

void SwerveManager::SetDelta(double D_Time)
{
    Del_Time = D_Time;
    for(int i = 0; i < Modules.size(); i++)
    {
        Modules.at(i)->SetDeltaTime(D_Time);
    }
}

void SwerveManager::ResetPID()
{
    for(int i = 0; i < Modules.size(); i++)
    {
        Modules.at(i)->ResetPID();
    }
}

void SwerveManager::ResetSwivelEnc()
{
    for(int i = 0; i < Modules.size(); i++)
    {
        Modules.at(i)->ResetSwivelEnc();
    }
}

void SwerveManager::ResetWheelEnc()
{
    for(int i = 0; i < Modules.size(); i++)
    {
        Modules.at(i)->ResetWheelEnc();
    }
}

double SwerveManager::Get()
{
    double Sum = 0;
    for(int i = 0; i < Modules.size(); i++)
    {
        Sum += Modules.at(i)->Get();
    }
    return Sum / Modules.size();
}

vector<SwerveModule*> SwerveManager::GetModules()
{
    return Modules;
}

void SwerveManager::DefaultSet()
{
    Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void SwerveManager::Set(DoubleSolenoid::Value value)
{
    Log::Error("WHY DID YOU CALL THE DOUBLESOLENOID SET FOR A MOTOR?!? Yell at your programmers!");
}