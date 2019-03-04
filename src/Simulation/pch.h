// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

#pragma once

// No C library depreciation warnings
#pragma warning ( disable : 4995 )
#pragma warning ( disable : 4996 )
#pragma warning ( disable : 4477 )

#define _CRT_SECURE_NO_WARNINGS
//end---adding our environment here------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <climits>
#include <stdio.h>
#include <cassert>
#include <math.h>
#include <windows.h>
#include <winbase.h>
#include <tchar.h>

#include <algorithm>
#include <functional>
//#include <xfunctional>
#include <memory>
#include <map>
#include <iostream>
#include <utility>
#include <array>
#include <deque>
#include <future>
#include <thread>
#include <chrono>

#include "RobotLibraries/Base/Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "RobotLibraries/Base/Vec2d.h"
#include "RobotLibraries/Base/Misc.h"
#include "RobotLibraries/Base/Event.h"
#include "RobotLibraries/Base/EventMap.h"
#include "RobotLibraries/Base/Script.h"
#include "RobotLibraries/Common/Entity_Properties.h"
#include "RobotLibraries/Common/Physics_1D.h"
#include "RobotLibraries/Common/Physics_2D.h"
#include "RobotLibraries/Common/Entity2D.h"
//#include "RobotLibraries/Common/Goal.h"
//#include "RobotLibraries/Common/Ship_1D.h"
//#include "RobotLibraries/Common/Ship.h"
//#include "RobotLibraries/Common/AI_Base_Controller.h"
//#include "RobotLibraries/Common/Vehicle_Drive.h"
//#include "RobotLibraries/Common/PIDController.h"
//#include "RobotLibraries/Common/Poly.h"
//#include "RobotLibraries/Common/Robot_Control_Interface.h"
//#include "RobotLibraries/Common/Calibration_Testing.h"
//#include "RobotLibraries/Common/Rotary_System.h"
//#include "RobotLibraries/Common/Servo_System.h"
//#include "RobotLibraries/Base/Joystick.h"
//#include "RobotLibraries/Base/JoystickBinder.h"
//#include "RobotLibraries/Common/UI_Controller.h"
//#include "RobotLibraries/Common/PIDController.h"
//#include "frc/WPILib.h"
#include "RobotLibraries/Base/Joystick.h"
#include "RobotLibraries/Base/JoystickBinder.h"
//#include "Common/InOut_Interface.h"
#include "RobotLibraries/Common/Debug.h"
//TODO enable robot control
#include "RobotLibraries/Common/Robot_Control_Common.h"
//#include "RobotLibraries/TankDrive/Tank_Robot2.h"
//This was depreciated and integrated ... stubbed out
// #include "TankDrive/src/Tank_Robot_Control.h"
//This isn't needed
// #include "TankDrive/src/Servo_Robot_Control.h"

//#include "RobotLibraries/FRC2019_Robot.h"
#include "RobotLibraries/Common/SmartDashboard.h"
