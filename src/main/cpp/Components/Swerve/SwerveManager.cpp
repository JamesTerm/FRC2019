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

SwerveManager::SwerveManager(string name, bool Wait, SwerveModule *FrontLeft, SwerveModule *FrontRight, SwerveModule *BackLeft, SwerveModule *BackRight) : OutputComponent(name)
{
    Modules.push_back(FrontLeft);
    Modules.push_back(BackLeft);
    Modules.push_back(FrontRight);
    Modules.push_back(BackRight);
    Pos = new double_Vector2();
    WaitSwivel = Wait;
}

SwerveManager::SwerveManager(string name, bool Wait, SwerveModule *FrontLeft, SwerveModule *FrontRight, SwerveModule *BackLeft, SwerveModule *BackRight, NavX *Nav) : OutputComponent(name)
{
    Modules.push_back(FrontLeft);
    Modules.push_back(BackLeft);
    Modules.push_back(FrontRight);
    Modules.push_back(BackRight);
    Pos = new double_Vector2();
    RobotNav = Nav;
    WaitSwivel = Wait;
}

SwerveManager::SwerveManager(string name, bool Wait, vector<SwerveModule*> Swerve_Modules, NavX *Nav) : OutputComponent(name)
{
    Modules = Swerve_Modules;
    Pos = new double_Vector2();
    RobotNav = Nav;
    WaitSwivel = Wait;
    OutputTable->PutNumber("Wheel", 0.018);
}

void SwerveManager::DeleteComponent()
{
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

void SwerveManager::UpdateLoc(double DirY, double DirX, double DirS)
{
    double joymag = sqrt((DirY * DirY) + (DirX * DirX));

    if (RobotNav != nullptr && joymag > 0)
    {
        double angle = ((atan2(DirY, DirX) * 180) / M_PI) - 90;
        if (angle < -180)
        {
            angle += 360;
        }
        if (abs(angle) == 180)
        {
            angle = 180;
        }

        double Heading = RobotNav->GetConstAngle();
        if (CheckSame())
        {
            Heading -= Modules.at(0)->GetSwivelTarget();
        }
        else
        {
            if(!CheckDiff())
            {
                Heading -= DiffHeading();
            }
            else
            {
                SwerveModule *AngleExt = GetExtreme(angle);    
                Heading -= AngleExt->GetSwivelTarget() - (DirS < 0 ? 90 : 0);
            }
        }
        LastHeading = Heading;
        OutputTable->PutNumber("Heading", Heading);
        
        double Mag = SwerveManager::GetEnc();
        double Change = (Mag - LastMag);
        LastMag = Mag;
        double RadHeading = (Heading * M_PI) / 180;

        double WheelAngle = (Change / Modules.at(0)->GetWheelTicks()) * 360;
        WheelAngle = (WheelAngle * M_PI) / 180;
        double WheelDiCal = OutputTable->GetNumber("Wheel", 0);
        double Dist = WheelAngle * (WheelDiCal / 2);

        Pos->X += -Dist * sin(RadHeading);
        Pos->Y += -Dist * cos(RadHeading);
    }
    else
    {
        OutputTable->PutNumber("Heading", 0);
        OutputTable->PutNumber("MagMove", 0);
    }

    OutputTable->PutNumber("Loc-X", Pos->X);
    OutputTable->PutNumber("Loc-Y", Pos->Y);
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
        SwerveManager::SetWheelAt(SwerveModule::Location::Front_Left, frontLeftSpeed);
        SwerveManager::SetWheelAt(SwerveModule::Location::Front_Right, frontRightSpeed);
        SwerveManager::SetWheelAt(SwerveModule::Location::Back_Left, backLeftSpeed);
        SwerveManager::SetWheelAt(SwerveModule::Location::Back_Right, backRightSpeed);
    }
    else
    {
        SwerveManager::Set(0);
        Log::General("Waiting for swivel motors to get their shit together");
    }
    SwerveManager::UpdateLoc(rawV, rawH, rawS);
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

bool SwerveManager::CheckSame()
{
    vector<SwerveModule*> SwivelTargets = SwerveManager::GetModules();
    double Tar = SwivelTargets.at(0)->GetSwivelTarget();

    for(int i = 0; i < SwivelTargets.size(); i++)
    {
        if (SwivelTargets.at(i)->GetSwivelTarget() != Tar)
            return false;
    }
    return true;
}

bool SwerveManager::CheckDiff()
{
    vector<SwerveModule*> SwivelTargets = SwerveManager::GetModules();

    for(int i = 0; i < SwivelTargets.size(); i++)
    {
        for(int j = 0; j < SwivelTargets.size(); j++)
        {
            if(j != i)
            {
                if (SwivelTargets.at(i)->GetSwivelTarget() == SwivelTargets.at(j)->GetSwivelTarget())
                {
                    return false;
                }
            }
        }
    }
    return true;
}

vector<SwerveModule*> SwerveManager::GetModules()
{
    return Modules;
}

vector<vector<SwerveModule*>> SwerveManager::GetGroups()
{
    vector<vector<SwerveModule*>> Groups;
    vector<SwerveModule*> SwivelTargets = SwerveManager::GetModules();
    vector<int> IndexsAdded;

    for(int a = 0; a < SwivelTargets.size(); a++)
    {
        vector<SwerveModule*> Pairs;
        if (!HasVal(a, IndexsAdded))
        {
            double Tar = SwivelTargets.at(a)->GetSwivelTarget();
            Pairs.push_back(SwivelTargets.at(a));
            for(int i = 0; i < SwivelTargets.size(); i++)
            {
                if (!HasVal(i, IndexsAdded))
                {
                    if(SwivelTargets.at(i)->GetSwivelTarget() == Tar)
                    {
                        Pairs.push_back(SwivelTargets.at(i));
                        IndexsAdded.push_back(i);
                    }
                }
            }
            Groups.push_back(Pairs);
        }
    }

    return Groups;
}

double SwerveManager::DiffHeading()
{
    vector<vector<SwerveModule*>> Headings = SwerveManager::GetGroups();
    vector<bool> PointingOut;
    double Heading = 0;
    for(int i = 0; i < Headings.size(); i++)
    {
        bool PointingOutbool = true;
        for(int j = 0; j < Headings.at(i).size(); j++)
        {
            PointingOutbool = PointingOutbool && !Headings.at(i).at(j)->IsPointingIn();
        }
        PointingOut.push_back(PointingOutbool);
    }

    bool AllOut = true;
    for(int i = 0; i < PointingOut.size(); i++)
    {
        AllOut = AllOut && PointingOut.at(i);
    }

    if(!AllOut)
    {
        for(int i = 0; i < PointingOut.size(); i++)
        {
            if (PointingOut.at(i))
            {
                Heading = Headings.at(i).at(0)->GetSwivelTarget();
            }
        }
    }

    return Heading;
}

bool SwerveManager::HasVal(int val, vector<int> V)
{
    for(int i = 0; i < V.size(); i++)
    {
        if (V.at(i) == val)
            return true;
    }
    return false;
}

SwerveModule* SwerveManager::GetExtreme(double forwardAngle)
{
    vector<SwerveModule*> Headings = SwerveManager::GetModules();
    double MaxAngle = -360;
    int index = 0;
    for(int i = 0; i < Headings.size(); i++)
    {
        if(!Headings.at(i)->IsPointingIn())
        {
            double AngleSwivel = CalNewAngle(Headings.at(i)->GetSwivelTarget(), forwardAngle);
            if((AngleSwivel) > MaxAngle)
            {
                index = i;
                MaxAngle = (AngleSwivel);
            }
        }
    }
    return Headings.at(index);
}

double SwerveManager::CalNewAngle(double OgAngle, double RotAngle)
{
    double NewA = OgAngle + RotAngle;
    if (NewA > 180)
    {
        NewA -= 360;
    }
    return NewA;
}

void SwerveManager::DefaultSet()
{
    Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void SwerveManager::Set(DoubleSolenoid::Value value)
{
    Log::Error("WHY DID YOU CALL THE DOUBLESOLENOID SET FOR A MOTOR?!? Yell at your programmers!");
}