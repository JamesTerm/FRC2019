/*
 * LeftScoreCube.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Dylann Ruiz
 */

#include <Autonomi/Timers/TimerSwitchLeft.h>
#include <UtilityFunctions.h>
#include <string>
#include <WPILib.h>
#include "Config/ActiveCollection.h"
#include "Config/Config.h"
#include "Components/VictorSPItem.h"

using namespace std;
using namespace Autonomi;

TimerSwitchLeft::TimerSwitchLeft(ActiveCollection *_activeCollection)
{
	activeCollection = _activeCollection;
}

void TimerSwitchLeft::Start()
{
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

	SetArm(-.15, activeCollection);

	DriveWithTimer(.25+error, .25, 3.9, activeCollection);

	if (gameData.length() > 0)
	{
		if (gameData[0] == 'L')
		{
			DriveWithTimer(.25+error, -.25, .75, activeCollection);

			DriveWithTimer(.25+error, .25, 2, activeCollection);

			StopArm(activeCollection);
			Wait(.25);

			DropCube(activeCollection);
		}
		else
		{
			DriveWithTimer(.25+error, .25, 5, activeCollection);
		}
	}
	else
	{
		DriveWithTimer(.25+error, .25, 5, activeCollection);
	}
}

