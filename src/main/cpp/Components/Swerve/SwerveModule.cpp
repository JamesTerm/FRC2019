/****************************** Header ******************************\
Class Name: SwerveModule
File Name:	SwerveModule.cpp
Summary: Module that holds the swivel motor and wheel motor
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/
#define _USE_MATH_DEFINES
#include <iostream>

#include "SwerveModule.h"
#include <cmath>

using namespace std;
using namespace frc;
using namespace Components;


SwerveModule::SwerveModule(string name, Motor *SwivelMtr, Motor *WheelMtr, EncoderItem* SwivelEnc, EncoderItem* WheelEnc, double TicksPerRev, double WheelTicks) : OutputComponent(name)
{

    GetType = SwerveModule::InputType::EncoderType;
    Swivel = SwivelMtr;
    Wheel = WheelMtr;

    SwivelEncoder = SwivelEnc;
    WheelEncoder = WheelEnc;

    EncRevTicks = TicksPerRev;
    WheelEncRevTicks = WheelTicks;
    SwerveModule::ResetEncs();
}

SwerveModule::SwerveModule(string name, Motor *SwivelMtr, Motor *WheelMtr, double TicksPerRev, double WheelTicks) : OutputComponent(name)
{
    Swivel = SwivelMtr;
    Wheel = WheelMtr;
    ((SparkMaxItem*)Wheel)->SetEncoderRev(WheelTicks);

    EncRevTicks = TicksPerRev;
    WheelEncRevTicks = WheelTicks;

    GetType = SwerveModule::InputType::MotorType;

    Swivel->GetPositionProfile()->SetBias(1000);
    Swivel->GetPositionProfile()->SetMaxChange(1);

/*
    OutputTable->PutNumber("SwivelP", 0);
    OutputTable->PutNumber("SwivelI", 0);
    OutputTable->PutNumber("SwivelD", 0);*/
    

    SwerveModule::ResetEncs();
}

void SwerveModule::DeleteComponent()
{
    delete this;
}

void SwerveModule::Set(double val)
{
    Wheel->Set(val * Dir);
}

void SwerveModule::SetSwivel(double SwivelVal)
{
    Swivel->Set(SwivelVal);
}

void SwerveModule::Set(double val, double SwivelVal)
{
    SwerveModule::Set(val);
    SwerveModule::SetSwivel(SwivelVal);
}

double SwerveModule::Get()
{
    return Wheel->Get();
}

double SwerveModule::GetSwivel()
{
    return Swivel->Get();
}

double SwerveModule::GetEnc()
{
    if (GetType == SwerveModule::InputType::EncoderType)
        return WheelEncoder->Get();
    else
        return ((SparkMaxItem*)Wheel)->GetEncoderValue();
}

double SwerveModule::GetSwivelEnc()
{
    if (GetType == SwerveModule::InputType::EncoderType)
        return SwivelEncoder->Get();
    else
        return ((TalonSRXItem*)Swivel)->GetQuadraturePosition();
}

void SwerveModule::ResetSwivelEnc()
{
    if (GetType == SwerveModule::InputType::EncoderType)
        SwivelEncoder->Reset();
    else
        ((TalonSRXItem*)Swivel)->SetQuadraturePosition(0);
}

void SwerveModule::ResetWheelEnc()
{
    if (GetType == SwerveModule::InputType::EncoderType)
        WheelEncoder->Reset();
    else
        ((SparkMaxItem*)Wheel)->Reset();
}

void SwerveModule::ResetEncs()
{
    SwerveModule::ResetSwivelEnc();
    SwerveModule::ResetWheelEnc();
}

void SwerveModule::UpdateWheelRate()
{
    double Pos = 0;
    if (GetType == SwerveModule::InputType::EncoderType)
    {
        Pos = WheelEncoder->Get();
    }
    else
    {
        Pos = ((SparkMaxItem*)Wheel)->GetEncoderValue();
    }

    double LastPos = Pos;
    if (LastWheelTick != LastPos)
    {
        Pos -= LastWheelTick;
        LastWheelTick = LastPos;
        LastChange = Pos;
    }
    else
    {
        Pos = LastChange;
    }
    
    Pos = abs(Pos) * Wheel->GetPowerProfile()->Sign(SwerveModule::Get());
    Pos /= WheelEncRevTicks;
    Pos *= WheelDi * M_PI;
    Pos /= D_Time;
    Pos *= abs(SwerveModule::Get()) > 0 ? 1 : 0;
    
    LastSpeed = Pos;
    OutputTable->PutNumber(name + "-Speed", LastSpeed);
}

void SwerveModule::ResetPID()
{
    Wheel->GetPositionProfile()->Reset();
    Swivel->GetPositionProfile()->Reset();
}

void SwerveModule::SetDeltaTime(double Time)
{
    D_Time = Time;
}

void SwerveModule::ProcessMotor(Motor *Subject, double Enc, double Target, double TickRev)
{
    double ValOut = -Subject->GetPositionProfile()->Calculate(Target, (Enc / TickRev) * 360, D_Time);
    //Log::General("----------------------------ValOut: " + to_string(ValOut) + " --------Target: " + to_string(Target) + " --------Current: " + to_string((Enc / TickRev) * 360) + " --------D_Time: " + to_string(D_Time));
    Subject->Set(ValOut);
}

bool SwerveModule::SetTargetSwivel(double Target)
{
    CurrentSwivelTarget = Target;
    SwerveModule::ProcessMotor(Swivel, SwerveModule::GetSwivelEnc(), Target, EncRevTicks);
    return (Swivel->GetPositionProfile()->ABSValue(Target - (SwerveModule::GetSwivelEnc() / EncRevTicks) * 360) > 45 ? Swivel->GetPositionProfile()->Inrange(Target, (SwerveModule::GetSwivelEnc() / EncRevTicks) * 360, 5) : true);
}

bool SwerveModule::SetTargetWheel(double Target)
{
    CurrentWheelTarget = Target;
    SwerveModule::ProcessMotor(Wheel, SwerveModule::GetEnc(), Target, WheelEncRevTicks);
    return Wheel->GetPositionProfile()->Inrange(Target, (SwerveModule::GetEnc() / EncRevTicks) * 360, 1);
}

bool SwerveModule::SetTarget(double Wheel_Target, double Swivel_Target)
{
    return SwerveModule::SetTargetSwivel(Swivel_Target) && SwerveModule::SetTargetWheel(Wheel_Target);
}

bool SwerveModule::SetSpeedTarget(double SPEEEED)
{
    SwerveModule::Set(Wheel->GetPowerProfile()->CalSpeed(SPEEEED, SwerveModule::Get(), SwerveModule::GetEnc(), D_Time));
    return Wheel->GetPowerProfile()->ReachedSpeed();
}

void SwerveModule::DefaultSet()
{
    Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void SwerveModule::Set(DoubleSolenoid::Value value)
{
    Log::Error("WHY DID YOU CALL THE DOUBLESOLENOID SET FOR A MOTOR?!? Yell at your programmers!");
}