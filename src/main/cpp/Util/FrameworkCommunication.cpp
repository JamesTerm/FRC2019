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

using namespace std;

//TODO: Check for memory leaks
namespace Util{
    
    class FrameworkCommunication{
        public:
            // Static access method
            static FrameworkCommunication& getInstance(){
                static FrameworkCommunication instance;
                return instance;
            }

            void SendData(string key, double value, bool announcePeriod = true){
                if(_IsAutononomous() && announcePeriod)
                    dashboardComm->PutNumber("AUTON_"+key,value);
                else if(_IsTeleoporated() && announcePeriod)
                    dashboardComm->PutNumber("TELEOP_"+key,value);
                else
                    dashboardComm->PutNumber(key,value);
            }

            void SendData(string k, int v, bool aP = true){
                SendData(k,(double)v,aP);
            }

            void SendData(string k, string v, bool aP = true){
                SendData(k,nt::Value::MakeString(v),aP);
            }

            void SendData(string k, bool v, bool aP = true){
                SendData(k,nt::Value::MakeBoolean(v),aP);
            }

            void SendHealthData(string key, string value){
                dashboardComm->PutValue("H_"+key,nt::Value::MakeString(value));
            }

            void SendHealthData(string key, double value){
                SendHealthData(key, to_string(value));
            }

        private:
            shared_ptr<nt::NetworkTable> dashboardComm;
            FrameworkCommunication();
            void SendData(string key, shared_ptr<nt::Value> value, bool announcePeriod = true){
                if(_IsAutononomous() && announcePeriod)
                    dashboardComm->PutValue("AUTON_"+key,value);
                else if(_IsTeleoporated() && announcePeriod)
                    dashboardComm->PutValue("TELEOP_"+key,value);
                else
                    dashboardComm->PutValue(key,value);
            }
        public:
            FrameworkCommunication(FrameworkCommunication const&) = delete;
            void operator=(FrameworkCommunication const&) = delete;

    };

    FrameworkCommunication::FrameworkCommunication(){
        auto inst = nt::NetworkTableInstance::GetDefault();
        dashboardComm = inst.GetTable(DASHBOARD_NETWORK_TABLE);
    }

}