/****************************** Header ******************************\
Class Name: DriveStraight [DEPRECATED]
File Name:	Drivestraight.cpp
Summary:	An autonomous program to make the robot drive straight
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylann Ruiz
Email: ruizdylann@gmail.com
\********************************************************************/

#include "DriveStraight.h"

using namespace std;
using namespace Autonomi;
using namespace Util;
//[DEPRECATED]
DriveStraight::DriveStraight(ActiveCollection *_activeCollection)
{
	activeCollection = _activeCollection;
}

void DriveStraight::Start()
{
	Log::General("drive straight");
	//DriveForward(500, .75, activeCollection);
	DriveWithTimer(.5,.5,2,activeCollection);
}