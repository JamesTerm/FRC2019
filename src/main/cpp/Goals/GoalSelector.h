/****************************** Header ******************************\
Class Name: none. Static Method SelectAuton
File Name: GoalSelector.h
Summary: Method to choose correct goal based on dashboard settings
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Chris Weeks
Email: chrisrweeks@aol.com
\********************************************************************/
#pragma once

#include "FRC2019_Goals.h"

static bool SelectAuton(ActiveCollection *activeCollection, MultitaskGoal *goal, string autonSelected, string positionSelected)
{
    bool isFound = true;
	if (autonSelected == "DEBUG")
	{
		//goal->AddGoal(new Goal_VisionAlign(activeCollection, new VisionTarget(320, 20), 140.0)); //!120 sec timeout for DEBUG only
        goal->AddGoal(new Goal_WaitThenDrive(activeCollection, .5, .5, 3, 5));
        cout << "selector" << endl;
		return true;
	}

    if(positionSelected == "Level 1 Left")
    {
        if(autonSelected == "DriveStraight")
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(6.0),.75));
        }
        else if(autonSelected == "OneHatchAuto")
        {
            //TODO
        }
        else
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(6.0),.75));
            isFound = false;
        }
    }
    else if(positionSelected == "Level 1 Center")
    {
        if(autonSelected == "DriveStraight")
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(6.0),.75));
        }
        else if(autonSelected == "OneHatchAuto")
        {
            //TODO
        }
        else
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(6.0),.75));
            isFound = false;
        }
    }
    else if(positionSelected == "Level 1 Right")
    {
        if(autonSelected == "DriveStraight")
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(6.0),.75));
        }
        else if(autonSelected == "OneHatchAuto")
        {
            //TODO
        }
        else
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(6.0),.75));
            isFound = false;
        }
    }
    else if(positionSelected == "Level 2 Left")
    {
        if(autonSelected == "DriveStraight")
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(10.0),.75));
        }
        else if(autonSelected == "OneHatchAuto")
        {
            //TODO
        }
        else
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(10.0),.75));
            isFound = false;
        }
    }
    else if(positionSelected == "Level 2 Right")
    {
        if(autonSelected == "DriveStraight")
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(10.0),.75));
        }
        else if(autonSelected == "OneHatchAuto")
        {
            //TODO
        }
        else
        {
            goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(10.0),.75));
            isFound = false;
        }
    }
    else
    {
        goal->AddGoal(new Goal_DriveStraight(activeCollection, new Feet(10.0),.75));
        isFound = false;
    }
    
    return isFound;
}
