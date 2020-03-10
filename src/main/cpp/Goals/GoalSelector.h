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

enum TeleOpGoal{
    eElevatorControl,
	eTimer,
    eDriveWithTimer,
    eRelativeElevatorControl,
    eShooter,
    eNone
};

static bool SelectAuton(ActiveCollection *activeCollection, MultitaskGoal *goal, string autonSelected, string positionSelected)
{
    bool isFound = true;
    
    //TODO: whatever you need to do here, if you're even using this method this year. You might need to edit params too.

    return isFound;
}

static Goal* SelectTeleOpGoal(ActiveCollection* activeCollection, TeleOpGoal goalSelected, double params){
	
    //TODO: whatever you need to do here, if you're even using this method this year. You might need to edit params too.
	return new Goal_TimeOut(activeCollection, params);
}