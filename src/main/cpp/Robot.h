/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
//#include <frc/WPILib.h>
#include <frc/RobotBase.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>

#include "Config/Config.h"
#include "Config/ActiveCollection.h"
#include "Systems/Drive.h"
#include "Controls/AxisControl.h"
#include "Autonomi/Autons.h"
#include "Util/Log.h"
#include "Goals/FRC2019_Goals.h"
#include "Goals/GoalSelector.h"
#include "Util/VisionTarget.h"
#include "Limelight/limelight.h"
#include "Util/UtilityFunctions.h"

using namespace frc;
using namespace System;
using namespace Controls;
using namespace Configuration;
using namespace Autonomi;
using namespace Logger;
using namespace Lime;

#define VERSION 1    //!< Defines the program version for the entire program.
#define REVISION "A" //!< Defines the revision of this version of the program.

class Robot : public RobotBase
{
      public:
        Robot();
		    ~Robot();

        void RobotInit();
        void Autonomous();
        void Teleop();
        void Test();
        void Disabled();

        void StartCompetition() override;
        void EndCompetition() override;

      private:
        Drive *m_drive;
        ActiveCollection *m_activeCollection; //!< Pointer to the only instantiation of the ActiveCollection Class in the program
        const string m_driveStraight = "drive";
        MultitaskGoal* m_masterGoal;
        MultitaskGoal* m_teleOpMasterGoal;
		nt::NetworkTableInstance m_inst; //!Network tables
        shared_ptr<NetworkTable> m_dashboardTable;
        string m_autonOptions[5] = {"DriveStraight","OnePieceAuto","TwoPieceAuto","DEBUG","NONE"};
        string m_positionOptions[5] = {"Level 1 Left", "Level 1 Center", "Level 1 Right", "Level 2 Left", "Level 2 Right"};
        std::atomic<bool> m_exit = false;
};
