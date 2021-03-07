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
#include "EncoderItem.h"

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

            EncoderItem* ConnectedEncoder = nullptr;
            EncoderItem* LinkedEncoder = nullptr;

            bool IsReversed = false;

        public:

            Motor() : OutputComponent(){}

            Motor(string name, bool reverse) : OutputComponent(name)
            {
                Motor::InitProfiles();
                IsReversed = reverse;
                LinkedEncoder = new EncoderItem(name + "-LinkedEncoder", this);
                ConnectedEncoder = LinkedEncoder;
            }

            void SetLinkedEncoder(EncoderItem* Encoder)
            {
                LinkedEncoder = Encoder;
            }

            EncoderItem* GetEncoder()
            {
                return LinkedEncoder;
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
                if(PositionProfile != nullptr)
                {
                    delete PositionProfile;
                }
                PositionProfile = new PIDProfile(Data);
            }

            void SetPowerProfile(ProfileData* Data)
            {
                if(PowerProfile != nullptr)
                {
                    delete PowerProfile;
                }
                PowerProfile = new PIDProfile(Data);
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
                if(ConnectedEncoder != nullptr)
                {
                    delete ConnectedEncoder;
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
                if(Power != 0)
                {
                    double Current = Get();
                    Current += -PowerProfile->Calculate(Power, Get(), Time) * (IsReversed ? -1 : 1);
                    Set(Current);
                }
                else
                {
                    Set(0);
                }
            }

            void SetPosition(double Target, double CurrentPosition, double Time)
            {
                Set(-PositionProfile->Calculate(Target, CurrentPosition, Time) * (IsReversed ? -1 : 1));
            }

            void SetPosition(double Target, double Time)
            {
                Set(-PositionProfile->Calculate(Target, LinkedEncoder->Get(), Time) * (IsReversed ? -1 : 1));
            }

            bool SetToPosition(double Target, double CurrentPosition, double Time)
            {
                SetPosition(Target, CurrentPosition, Time);
                return PositionProfile->Inrange(Target, CurrentPosition, PositionProfile->GetThres());
            }

            bool SetToPosition(double Target, double Time)
            {
                SetPosition(Target, LinkedEncoder->Get(), Time);
                return PositionProfile->Inrange(Target, LinkedEncoder->Get(), PositionProfile->GetThres());
            }

            void SetSpeed(double Rate, double CurrentPosition, double Time)
            {
                Set(-PowerProfile->CalSpeed(Rate, Get(), CurrentPosition, Time) * (IsReversed ? -1 : 1));
            }

            void SetSpeed(double Rate, double Time)
            {
                Set(-PowerProfile->CalSpeed(Rate, Get(), LinkedEncoder->Get(), Time) * (IsReversed ? -1 : 1));
            }

            bool SetToSpeed(double Rate, double CurrentPosition, double Time)
            {
                SetSpeed(Rate, CurrentPosition, Time);
                return PowerProfile->ReachedSpeed();
            }

            bool SetToSpeed(double Rate, double Time)
            {
                SetSpeed(Rate, LinkedEncoder->Get(), Time);
                return PowerProfile->ReachedSpeed();
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