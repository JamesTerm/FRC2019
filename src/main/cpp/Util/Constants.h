/****************************** Header ******************************\
Class Name: Constants
File Name: Constants.h
Summary: File of constant values to be used throughout the program.
Project: FRC2019CPP
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com
\********************************************************************/

#ifndef SRC_UTIL_CONSTANTS_H_
#define SRC_UTIL_CONSTANTS_H_

#include <cmath>
#include <limits>

using namespace std;

namespace Util
{
	static const double MINIMUM_JOYSTICK_RETURN = 0.04;
	static const double EPSILON_MIN = std::numeric_limits<double>::epsilon();

	//* For Dashboard
	static const string DASHBOARD_NETWORK_TABLE = "DASHBOARD_2017";
	static const string AUTON = "AUTON";
	static const string TELEOP = "TELEOP";
	static const string DISABLED = "DISABLED";

	
	static const double TICKS_PER_INCH = 5.0; //made up number right now
}

#endif /* SRC_UTIL_CONSTANTS_H_ */
