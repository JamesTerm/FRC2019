/****************************** Header ******************************\
Class Name: VisionTarget
File Name: VisionTarget.cpp
Summary: Utility class to manipulate data relating to vision targets
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson, Chris Weeks
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com, 
chrisrweeks@aol.com
\********************************************************************/

#pragma once

#include <string>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"


using namespace std;

class VisionTarget
{
  public:
    VisionTarget(int _x, int _y, int _radius)
    {
        X = _x;
        Y = _y;
        Radius = _radius;
    }
    VisionTarget(int _x, int _radius) : VisionTarget(_x, 0, _radius) {}
    VisionTarget() : VisionTarget(0, 0, 0) {}

    const int getX() { return X; }
    const int getY() { return Y; }
    const int getRadius() { return Radius; }

    const int setX(int _x) 
	{ 
		X = _x; 
		return X;
	}
    const int setY(int _y) 
	{ 
		Y = _y; 
		return Y;
	}
    const int setRadius(int _radius) 
	{ 
		Radius = _radius; 
		return Radius;
	}
    const int compareX(VisionTarget *target)
    {
        return -(X - target->getX());
    }
    const int compareY(VisionTarget *target)
    {
        return -(Y - target->getY());
    }
    const int compareRadius(VisionTarget *target)
    {
        return (Radius - target->getRadius());
    }

  private:
    int X, Y, Radius;
};