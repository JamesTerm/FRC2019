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

#include "FRC_Goals.h"

enum TeleOpGoal{
    eElevatorControl,
	eTimer,
    eDriveWithTimer,
    eRelativeElevatorControl,
    eShooter,
    eMotorPosition,
    eNone
};

static bool SelectAuton(ActiveCollection *activeCollection, MultitaskGoal *goal, string autonSelected, string positionSelected)
{
    bool isFound = true;
    
    //TODO: whatever you need to do here, if you're even using this method this year. You might need to edit params too.

    return isFound;
}

static Goal* SelectTeleOpGoal(ActiveCollection* activeCollection, TeleOpGoal goalSelected, double params = 0){
	/*if (goalSelected == TeleOpGoal::eElevatorControl)
		return new Goal_ElevatorControl(activeCollection, params);
	else*/ if (goalSelected == TeleOpGoal::eTimer)
		return new Goal_TimeOut(activeCollection, params);
    else if(goalSelected == TeleOpGoal::eDriveWithTimer)
        return new Goal_DriveWithTimer(activeCollection, .5, .5, params);
    /*else if(goalSelected == TeleOpGoal::eRelativeElevatorControl)
        return new Goal_RelativeElevatorControl(activeCollection, params);*/
    else if(goalSelected == TeleOpGoal::eShooter)
        return new Goal_ShooterBunch(activeCollection, params);
    else if(goalSelected == TeleOpGoal::eMotorPosition)
        return new Goal_MotorPosition(activeCollection);
	return new Goal_TimeOut(activeCollection, params);
}