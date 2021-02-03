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

#include <iostream>

#include "SwerveModule.h"

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

    WheelPID = new PIDProfile(1, 0, 0);
    SwivelPID = new PIDProfile(1, 0, 0);
    SpeedPID = new PIDProfile(1, 0, 0);

    SwerveModule::ResetEncs();
}

SwerveModule::SwerveModule(string name, Motor *SwivelMtr, Motor *WheelMtr, double TicksPerRev, double WheelTicks) : OutputComponent(name)
{
    Swivel = SwivelMtr;
    Wheel = WheelMtr;

    EncRevTicks = TicksPerRev;
    WheelEncRevTicks = WheelTicks;

    GetType = SwerveModule::InputType::MotorType;

    WheelPID = new PIDProfile(0.2, 0, 0.002);
    SwivelPID = new PIDProfile(0.8, 0.01, 0);
    SwivelPID->SetBias(1000);
    SpeedPID = new PIDProfile(0.2, 0, 0.002);
/*
    OutputTable->PutNumber("SwivelP", 0);
    OutputTable->PutNumber("SwivelI", 0);
    OutputTable->PutNumber("SwivelD", 0);*/
    

    SwerveModule::ResetEncs();
}

void SwerveModule::DeleteComponent()
{
    delete WheelPID;
    delete SwivelPID;
    delete SpeedPID;
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


void SwerveModule::ResetPID()
{
    WheelPID->Reset();
    SwivelPID->Reset();
}

void SwerveModule::SetDeltaTime(double Time)
{
    D_Time = Time;
}

void SwerveModule::ProcessMotor(Motor *Subject, double Enc, PIDProfile *Profile, double Target, double TickRev)
{
    double ValOut = -Profile->Calculate(Target, (Enc / TickRev) * 360, D_Time);
    //Log::General("----------------------------ValOut: " + to_string(ValOut) + " --------Target: " + to_string(Target) + " --------Current: " + to_string((Enc / TickRev) * 360) + " --------D_Time: " + to_string(D_Time));
    Subject->Set(ValOut);
}

bool SwerveModule::SetTargetSwivel(double Target)
{
    CurrentSwivelTarget = Target;
    SwerveModule::ProcessMotor(Swivel, SwerveModule::GetSwivelEnc(), SwivelPID, Target, EncRevTicks);
    return (SwivelPID->ABSValue(Target - (SwerveModule::GetSwivelEnc() / EncRevTicks) * 360) > 45 ? SwivelPID->Inrange(Target, (SwerveModule::GetSwivelEnc() / EncRevTicks) * 360, 5) : true);
}

bool SwerveModule::SetTargetWheel(double Target)
{
    Target *= Dir;
    CurrentWheelTarget = Target;
    SwerveModule::ProcessMotor(Wheel, SwerveModule::GetEnc(), WheelPID, Target, WheelEncRevTicks);
    return WheelPID->Inrange(Target, (SwerveModule::GetEnc() / EncRevTicks) * 360, 1);
}

bool SwerveModule::SetTarget(double Wheel_Target, double Swivel_Target)
{
    return SwerveModule::SetTargetSwivel(Swivel_Target) && SwerveModule::SetTargetWheel(Wheel_Target);
}

bool SwerveModule::SetSpeedTarget(double SPEEEED)
{
    SPEEEED *= Dir;
    SwerveModule::Set(WheelPID->CalSpeed(SPEEEED, SwerveModule::Get(), SwerveModule::GetEnc(), D_Time));
    return WheelPID->ReachedSpeed();
}

void SwerveModule::DefaultSet()
{
    Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A MOTOR?!? Yell at your programmers!");
}

void SwerveModule::Set(DoubleSolenoid::Value value)
{
    Log::Error("WHY DID YOU CALL THE DOUBLESOLENOID SET FOR A MOTOR?!? Yell at your programmers!");
}