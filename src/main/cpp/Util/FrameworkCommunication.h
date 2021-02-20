#pragma once

#include <networktables/NetworkTableEntry.h>
#include "LoopChecks.h"

using namespace std;

namespace Util
{
class FrameworkCommunication
{
public:
    static FrameworkCommunication &GetInstance()
    {
        static FrameworkCommunication instance;
        return instance;
    }

    void SendData(string key, double value, bool announcePeriod = true)
    {
        nt::NetworkTableEntry ent;
        if (_IsAutononomous() && announcePeriod)
        {
            dashboardComm->PutNumber("AUTON_" + key, value);
            ent = dashboardComm->GetEntry("AUTON_" + key);
        }
        else if (_IsTeleoporated() && announcePeriod)
        {
            dashboardComm->PutNumber("TELEOP_" + key, value);
            ent = dashboardComm->GetEntry("TELEOP_" + key);
        }
        else
        {
            dashboardComm->PutNumber(key, value);
            ent = dashboardComm->GetEntry(key);
        }
        ent.ClearPersistent();
    }

    void SendData(string k, int v, bool aP = true, bool isBool = true)
    {
        if (isBool)
            SendData(k, nt::Value::MakeString((v ? "true" : "false")), aP);
        else
            SendData(k, (double)v, aP);
    }

    void SendData(string k, string v, bool aP = true)
    {
        SendData(k, nt::Value::MakeString(v), aP);
    }

    //! DEPRECATED; USE SENDDATA(STRING,INT,BOOL,BOOL) INSTEAD
    /*void SendData(string k, bool v, bool aP = true){
                SendData(k,nt::Value::MakeString(to_string(v)),aP);
            }*/

    void SendHealthData(string key, string value)
    {
        nt::NetworkTableEntry ent;
        dashboardComm->PutValue("H_" + key, nt::Value::MakeString(value));
        ent = dashboardComm->GetEntry("H_" + key);
        ent.ClearPersistent();
    }

    void SendHealthData(string key, double value)
    {
        SendHealthData(key, to_string(value));
    }

    void NotifyRobotState(string value)
    {
        nt::NetworkTableEntry ent;
        dashboardComm->PutValue("ROBOT_STATE", nt::Value::MakeString(value));
        ent = dashboardComm->GetEntry("ROBOT_STATE");
        ent.ClearPersistent();
    }

private:
    shared_ptr<nt::NetworkTable> dashboardComm;
    FrameworkCommunication();
    void SendData(string key, shared_ptr<nt::Value> value, bool announcePeriod = true)
    {
        nt::NetworkTableEntry ent;
        if (_IsAutononomous() && announcePeriod)
        {
            dashboardComm->PutValue("AUTON_" + key, value);
            ent = dashboardComm->GetEntry("AUTON_" + key);
        }
        else if (_IsTeleoporated() && announcePeriod)
        {
            dashboardComm->PutValue("TELEOP_" + key, value);
            ent = dashboardComm->GetEntry("TELEOP_" + key);
        }
        else
        {
            dashboardComm->PutValue(key, value);
            ent = dashboardComm->GetEntry(key);
        }
        ent.ClearPersistent();
    }

public:
    FrameworkCommunication(FrameworkCommunication const &) = delete;
    void operator=(FrameworkCommunication const &) = delete;
};
} // namespace Util