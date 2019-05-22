/****************************** Header ******************************\
Class Name: DriveStraight [DEPRECATED]
File Name:	Drivestraight.h
Summary:	An autonomous program to make the robot drive straight
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylann Ruiz
Email: ruizdylann@gmail.com
\********************************************************************/
#ifndef SRC_AUTONOMI_DRIVESTRAIGHT_H_
#define SRC_AUTONOMI_DRIVESTRAIGHT_H_

#include <string>
#include <frc/WPILib.h>
#include <fstream>
#include <thread>
#include <iostream>

#include "../Config/ActiveCollection.h"
#include "../Util/UtilityFunctions.h"
#include "../Config/ActiveCollection.h"
#include "../Config/Config.h"
#include "../Components/VictorSPItem.h"

using namespace std;
using namespace Configuration;
//[DEPRECATED]
namespace Autonomi{
class DriveStraight
{
	private:
		ActiveCollection *activeCollection;
		double error = .01;
		string gameData;

	public:
		DriveStraight(ActiveCollection *_activeCollection);
		void Start();
		virtual ~DriveStraight(){};
};
}

#endif /* SRC_AUTONOMI_DRIVESTRAIGHT_H_ */