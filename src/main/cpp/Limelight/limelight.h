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
#pragma once

#ifndef SRC_limelight_H_
#define SRC_limelight_H_

#ifdef _Win32
#include "../Util//SmartDashboard.h"
#else
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "../Global.h"
#endif
#include "cmath"
#include "../Components/NativeComponent.h"
#include "iostream"

using namespace std;
using namespace Components;
namespace Lime{
class limelight : public NativeComponent //Inheritance or something 
{
    public:
        limelight() : NativeComponent("LimeLight")
        {
            SetLED();
            SetCamMode();
            SetPipeline();
        } 
        double HorizontalOffset()
        {
            if(ProcessTarget && HasLEDon)
            {
                return table->GetNumber("tx", 0.0);
            }
            else
            {
                Log::Error("Turn on the led and turn on vision processing in order to get target");
                return 0;
            }
        }
        double VerticalOffset()
        {
            if(ProcessTarget && HasLEDon)
            {
                return (table->GetNumber("ty", 0.0));
            }
            else
            {
                Log::Error("Turn on the led and turn on vision processing in order to get target");
                return 0;
            }
        }
        double TargetDistance(double TargetHeight)
        {
            if(ProcessTarget && HasLEDon)
            {
                double robotheight = TargetHeight - 42;
                double radianAngle = (tan(((VerticalOffset()) * 3.1415) / 180));
                Log::General("angle: " + to_string(radianAngle) + "  :: Limelight input: " + to_string(VerticalOffset()));
                return (robotheight / (radianAngle)) + 5;
            }
            else
            {
                Log::Error("Turn on the led and turn on vision processing in order to get target");
                return 0;
            }
        }
        bool SeesTarget()
        {
            if(ProcessTarget && HasLEDon)
            {
                int Present = table->GetNumber("tv", 0.0);
                if(Present > 0)
                {
                    return true;
                }
                return false;
            }
            else
            {
                Log::Error("Turn on the led and turn on vision processing in order to get target");
                return false;
            }
        }
        void SetPipeline(int Pipe = 0)
        {
            if(Pipe >= 0 && Pipe < 10)
                table->PutNumber("pipeline", Pipe);
            else
                Log::Error("Pipelines only go from 0 - 9 not: " + to_string(Pipe));
        }
        void SetCamMode(bool VisionProcess = true)
        {
            if(VisionProcess)
            {
                ProcessTarget = true;
                table->PutNumber("camMode", 0);
            }
            else
            {
                ProcessTarget = true;
                table->PutNumber("camMode", 1);
            }    
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
        void SetLED(int State = 0)
        {
            if(State == 0)
            {
                HasLEDon = true;
                table->PutNumber("ledMode", 3);
            }
            else if(State == 1)
            {
                HasLEDon = false;
                table->PutNumber("ledMode", 1);
            }
            else
            {
                HasLEDon = false;
                table->PutNumber("ledMode", 2);
                Log::Error("THE ROBOT IS KILLING PEOPLE'S EYES, KEEP IT UP AND SOMEONE MIGHT NOTICE!");
            }
        }
    private:
        bool HasLEDon = true;
        bool ProcessTarget = true;
        std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
 };
}
#endif /* SRC_limelight_H_ */