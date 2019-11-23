/****************************** Header ******************************\
Class Name: limelight
File Name:	limelight.h
Summary: Limelight header file and to get limelight values
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#ifndef SRC_limelight_H_
#define SRC_limelight_H_

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "../Global.h"

using namespace std;
namespace Lime{
class limelight
{
    public:
        double HorizontalOffset()
        {
          Log::General(to_string(table->GetNumber("tx", 0.0)) + "", true);
          return table->GetNumber("tx", 0.0);
        }
        double VerticalOffset()
        {
            Log::General(to_string(table->GetNumber("ty", 0.0)), true);
            return table->GetNumber("ty", 0.0);
        }
        double TargetDistance()
        {
            Log::General(to_string(table->GetNumber("ta", 0.0)), true);
            return table->GetNumber("ta", 0.0);
        }
        bool SeesTarget()
        {
            Log::General(to_string(table->GetNumber("tv", 0.0)), true);
            int Present = table->GetNumber("tv", 0.0);
            if(Present > 0)
            {
                return true;
            }
            return false;
        }
        void SetCamMode(int Mode)
        {
            table->PutNumber("camMode", Mode);
        }
        void Snapshots(bool takeShot)
        {
            if(takeShot)
            {
                table->PutNumber("snapshot", 1);
            }
            else
            {
                table->PutNumber("snapshot", 0);
            }
        }
        void SetLED(bool On)
        {
            if(On)
            {
                table->PutNumber("ledMode", 0);
            }
            else
            {
                table->PutNumber("ledMode", 1);
            }
        }
    private:
        std::shared_ptr<NetworkTable> table = table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
 };
}
#endif /* SRC_limelight_H_ */