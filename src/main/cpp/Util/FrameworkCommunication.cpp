/****************************** Header ******************************\
Class Name: FrameworkCommunication [Singleton]
File Name: FrameworkCommunication.cpp
Summary: Class to manage communication between the code and the custom 
         WinForms dashboard.
Project: FRC2020
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com
Special thanks to pazdera on GitHub for the inspiration on Singleton mechanics
\********************************************************************/

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include "Constants.h"
#include "LoopChecks.h"
#include "RobotStatus.h"
#include "FrameworkCommunication.h"

//TODO: Check for memory leaks
namespace Util{

    auto Instance_RobotStatusChanged = [&](EventArgs* e){
        try{
            auto args = (TEventArgs<RobotState, RobotStatus*>*)e;
            RobotState state = args->GetValue();
            switch(state){
                case RobotState::Auton: FrameworkCommunication::GetInstance().NotifyRobotState(AUTON); break;
                case RobotState::Teleop: FrameworkCommunication::GetInstance().NotifyRobotState(TELEOP); break;
                default: FrameworkCommunication::GetInstance().NotifyRobotState(DISABLED); break;

            }
        }catch(exception &e){
            Log::Error("Known Exception Thrown in Instance_RobotStatusChanged in a Control! This can cause fatal Runtime Errors! Check your logs.");
            SmartDashboard::PutString("Instance_RobotStatusChanged", "Error");
            //TODO: Make this the append instead
            Log::Error(e.what());
        }catch(...){
            Log::Error("UnknownException Thrown in Instance_RobotStatusChanged in a Control! This can cause fatal Runtime Errors! Check your logs and yell at the programmers!");
            SmartDashboard::PutString("Instance_RobotStatusChanged", "Error");
        }
    };

    FrameworkCommunication::FrameworkCommunication(){
        RobotStatus::GetInstance().RobotStatusChanged += Instance_RobotStatusChanged;
        auto inst = nt::NetworkTableInstance::GetDefault();
        dashboardComm = inst.GetTable(DASHBOARD_NETWORK_TABLE);
    }

}