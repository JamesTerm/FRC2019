/****************************** Header ******************************\
Class Name: ServoItem inherits OutputComponent
File Name:	ServoItem.cpp
Summary: Servo controller
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot983@gmail.com
\********************************************************************/

#include <iostream>

#include "ServoItem.h"

using namespace std;
using namespace frc;
using namespace Components;

ServoItem::ServoItem(string name, int port, double MaxAngle, ServoType Type, bool Real) : OutputComponent(name)
{
    Angle = MaxAngle;
    BotServo = new Servo(port);
    ServoCal = Type;
    CurrentPercent = BotServo->Get();
    FromTable(Real);
	{
		Log::General("Using Table values");
		OutputTable->PutNumber(name + "-Angle", CurrentPercent);
	}
}

void ServoItem::Set(double val)
{
    CurrentPercent = val;
    if(ServoCal == ServoType::Limited)
    {
        BotServo->Set(val);
    }
    else
    {
        BotServo->Set((val + 1) / 2);
    }
}

void ServoItem::AngleSet(double angle)
{
    ServoItem::Set(abs(angle / Angle));
}

void ServoItem::SetOffline()
{
    CurrentPercent = 0;
    BotServo->SetOffline();
}

double ServoItem::Get()
{
    return CurrentPercent * Angle;
}

void ServoItem::DefaultSet()
{
    Log::Error("WHY DID YOU CALL THE DEFAULT SET FOR A SERVO?!? Yell at your programmers! Retard (or don't, its really up to you)");
}

void ServoItem::Set(DoubleSolenoid::Value value)
{
    Log::Error("WHY DID YOU CALL SET SOLENOID FOR A SERVO?!? Yell at your programmers! Retard (or don't, its really up to you)");
}

void ServoItem::DeleteComponent()
{
    delete BotServo;
}

void ServoItem::UpdateComponent()
{
    if (!UseTable)
	{
		OutputTable->PutNumber(name + "-Angle", ServoCal == ServoType::Limited? ServoItem::Get() : CurrentPercent);
	}
}