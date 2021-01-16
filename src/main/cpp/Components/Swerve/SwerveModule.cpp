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

void SwerveModule::DeleteComponent()
{
    delete WheelPID;
    delete SwivelPID;
    delete SpeedPID;
    delete this;
}

void SwerveModule::Set(double val)
{
    Wheel->Set(val);
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
    return WheelEncoder->Get();
}

double SwerveModule::GetSwivelEnc()
{
    return SwivelEncoder->Get();
}

void SwerveModule::ResetSwivelEnc()
{
    SwivelEncoder->Reset();
}

void SwerveModule::ResetWheelEnc()
{
    WheelEncoder->Reset();
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

void SwerveModule::ProcessMotor(Motor *Subject, EncoderItem *Enc, PIDProfile *Profile, double Target, double TickRev)
{
    Subject->Set(Profile->Calculate(Target, (Enc->Get() / TickRev) * 360, D_Time));
}

bool SwerveModule::SetTargetSwivel(double Target)
{
    SwerveModule::ProcessMotor(Swivel, SwivelEncoder, SwivelPID, Target, EncRevTicks);
    return SwivelPID->Inrange(Target, SwivelEncoder->Get(), 0.1);
}

bool SwerveModule::SetTargetWheel(double Target)
{
    SwerveModule::ProcessMotor(Wheel, WheelEncoder, WheelPID, Target, WheelEncRevTicks);
    return WheelPID->Inrange(Target, WheelEncoder->Get(), 0.1);
}

bool SwerveModule::SetTarget(double Wheel_Target, double Swivel_Target)
{
    return SwerveModule::SetTargetSwivel(Swivel_Target) && SwerveModule::SetTargetWheel(Wheel_Target);
}

bool SwerveModule::SetSpeedTarget(double SPEEEED)
{
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