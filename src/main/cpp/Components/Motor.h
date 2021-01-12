/****************************** Header ******************************\
Class Name: motor
File Name:	motor.h
Summary: Abstract class to motor
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#ifndef SRC_Motor_motor_H_
#define SRC_Motor_motor_H_

#include "ctre/Phoenix.h"
#include "OutputComponent.h"

#include <frc/RobotBase.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>


using namespace std;
using namespace frc;
using namespace Util;

namespace Components
{
    class Motor : public OutputComponent
    {
        private:
            int PDBPort = 0;
            double TimeTimedOut = 0;
            double LowerAmount = 0;
            std::shared_ptr<NetworkTable> MotorTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
            double ABSVal(double val)
            {
                return (val < 0 ? val * -1 : val);
            }

            int SignVal(double val)
            {
                return (val >= 0 ? 1 : -1);
            }
            double RegenRate = 0.01;
            double PersonalLowerRate = 0;

        public:

            Motor() : OutputComponent(){}
			Motor(string name) : OutputComponent(name){  }

            void SetPDBChannel(int val) { PDBPort = val; }
            int GetPDBChannel() {return PDBPort;}

            void SetRegenRate(double Rate) {RegenRate = Rate;}
            void SetLowerRate(double Rate) {PersonalLowerRate = Rate;}

            void SetTimeOut(double Time, double lowerAmount) {TimeTimedOut = Time; LowerAmount += (PersonalLowerRate == 0 ? lowerAmount : PersonalLowerRate);}
            double CalculateVal(double val)
            {
                double ReturnVal = val;
                if (val != 0)
                {
                    if (TimeTimedOut > 0)
                    {
                        TimeTimedOut -= 0.1;
                    }
                    else
                    {
                        if (LowerAmount > 0)
                            LowerAmount -= RegenRate;
                        if (LowerAmount < 0)
                            LowerAmount = 0;
                    }

                    double NewVal = ABSVal(val) - ABSVal(LowerAmount);
                    if (NewVal < 0)
                    {
                        NewVal = 0;
                    }
                    ReturnVal = NewVal * SignVal(val);
                }
                else
                {
                    TimeTimedOut = 0;
                    LowerAmount = 0;
                }
                MotorTable->PutNumber(name, ReturnVal);
                Log::General(name + " : Power->" + to_string(ReturnVal));
                return ReturnVal;
            }

            virtual double Get() = 0;
            virtual void Set(DoubleSolenoid::Value value) = 0;
            virtual void Set(double val) = 0;
            virtual void DefaultSet() = 0;
            virtual void Stop() = 0;
    };
}

#endif /* SRC_Motor_motor_H_ */