/****************************** Header ******************************\
Class Name: Distance and its subclasses
File Name: Distances.h
Summary: Distance objects for ease of use when using distances in
program. Each object can be converted to any other object
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson, Chris Weeks
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com, 
chrisrweeks@aol.com
\********************************************************************/
#pragma once
#include "../Constants.h"
using namespace Util;
class Distance
{
    public:
    const double toEncoderTicks() {return m_value * inATick();}
    const double toInches() {return m_value * inAnInch();}
    const double toFeet() {return m_value * inAFoot();}
    const double toYards() {return m_value * inAYard();}

    //TODO make to{Distance}Object() work
    // const EncoderTicks* toEncoderTicksObject() {return new EncoderTicks(toEncoderTicks());}
    // const Inches* toInchesObject() {return new Inches(toInches());}
    // const Feet* toFeetObject() {return new Feet(toFeet());}
    // const Yards* toYardsObject() {return new Yards(toYards());}

    virtual double inATick() = 0;

    const double inAnInch() {return inATick() * TICKS_PER_INCH;}
    const double inAFoot() {return inAnInch() * 12.0;}
    const double inAYard() {return inAFoot() * 3.0;}
    protected:
    double m_value;
};

class EncoderTicks : public Distance
{
    public:
    EncoderTicks(double value) {m_value = value;}
    
    double inATick() {return 1.0;}
};

class Inches : public Distance
{
    public:
    Inches(double value) {m_value = value;}

    double inATick() {return 1.0 / TICKS_PER_INCH;}

};
class Feet : public Distance
{
    public:
    Feet(double value) {m_value = value;}
    double inATick() {return 1.0 / 12.0 / TICKS_PER_INCH;}
};

class Yards : public Distance
{
    public:
    Yards(double value) {m_value = value;}
    double inATick() {return 1.0 / 3.0 / 12.0 / TICKS_PER_INCH;}
};