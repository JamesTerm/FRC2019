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
#include "../Util/PIDProfile.h"


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
            double CurrentPowerTarget = 0;
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

            PIDProfile *PositionProfile = nullptr;
            PIDProfile *PowerProfile = nullptr;

        public:

            Motor() : OutputComponent(){}
			Motor(string name) : OutputComponent(name)
            { 
                Motor::InitProfiles();
            }

            void SetPDBChannel(int val) { PDBPort = val; }
            int GetPDBChannel() {return PDBPort;}

            void InitProfiles(double Ppos = 0.8, double Ipos = 0.01, double Dpos = 0, double Pspe = 3, double Ispe = 0, double Dspe = 0)
            {
                PositionProfile = new PIDProfile(Ppos, Ipos, Dpos);
                PowerProfile = new PIDProfile(Pspe, Ispe, Dspe);
                PowerProfile->SetMaxChange(0.15);
            }

            void SetPositonProfile(double Ppos = 0.8, double Ipos = 0.01, double Dpos = 0, double Bias = -1)
            {
                if(PositionProfile != nullptr)
                {
                    delete PositionProfile;
                }
                PositionProfile = new PIDProfile(Ppos, Ipos, Dpos, (Bias < 0 ? Ppos * 100 : Bias));
            }

            void SetPowerProfile(double Ppos = 0.8, double Ipos = 0.01, double Dpos = 0, double Bias = -1)
            {
                if(PowerProfile != nullptr)
                {
                    delete PowerProfile;
                }
                PowerProfile = new PIDProfile(Ppos, Ipos, Dpos, (Bias < 0 ? Ppos * 100 : Bias));
            }

            void SetPositonProfile(ProfileData* Data)
            {
                Motor::SetPositonProfile(Data->Pval, Data->Ival, Data->Dval, Data->Bias);
            }

            void SetPowerProfile(ProfileData* Data)
            {
                Motor::SetPowerProfile(Data->Pval, Data->Ival, Data->Dval, Data->Bias);
            }

            void CleanUpProfiles()
            {
                if(PositionProfile != nullptr)
                {
                    delete PositionProfile;
                }
                if(PowerProfile != nullptr)
                {
                    delete PowerProfile;
                }
            }

            PIDProfile* GetPositionProfile(){return PositionProfile;};
            PIDProfile* GetPowerProfile(){return PowerProfile;};

            void SetPower(double Power, double Time)
            {
                if(Power != CurrentPowerTarget)
                {
                    CurrentPowerTarget = Power;
                    PowerProfile->Reset();
                }
                double Current = Get();
                Current += -PowerProfile->Calculate(Power, Get(), Time);
                Set(Current);
            }

            void SetPosition(double Target, double CurrentPosition, double Time)
            {
                Set(PositionProfile->Calculate(Target, CurrentPosition, Time));
            }

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
                OutputTable->PutNumber(name, ReturnVal);
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