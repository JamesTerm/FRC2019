/****************************** Header ******************************\
Class Name: SwerveModule
File Name:	SwerveModule.h
Summary: Module that holds the swivel motor and wheel motor
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#ifndef SRC_SWERVE_MODULE_H_
#define SRC_SWERVE_MODULE_H_

#include "../Motor.h"
#include "../EncoderItem.h"

using namespace std;
using namespace frc;
using namespace Util;

namespace Components
{
    class SwerveModule : public OutputComponent
    {
        public:
            SwerveModule(string name, Motor *SwivelMtr, Motor *WheelMtr, EncoderItem* SwivelEnc, EncoderItem* WheelEnc, double TicksPerRev, double WheelTicks);
            
            void Set(double val, double SwivelVal);
            void SetSwivel(double SwivelVal);
            double GetSwivel();
            virtual void Set(double val);
			virtual double Get();

            double GetEnc();
            double GetSwivelEnc();

            void ResetEncs();
            void ResetWheelEnc();
            void ResetSwivelEnc();

            void ResetPID();
            void SetDeltaTime(double Time);
            void ProcessMotor(Motor *Subject, EncoderItem *Enc, PIDProfile *Profile, double Target, double TickRev);
            bool SetTargetWheel(double Target);
            bool SetTargetSwivel(double Target);
            bool SetTarget(double Wheel_Target, double Swivel_Target);
            bool SetSpeedTarget(double SPEEEEED);

            virtual void DefaultSet();
			virtual void Set(DoubleSolenoid::Value value);
            virtual void DeleteComponent() override;

            PIDProfile *WheelPID;
            PIDProfile *SwivelPID;
            PIDProfile *SpeedPID;

        private:
            Motor *Swivel;
            Motor *Wheel;
            EncoderItem *SwivelEncoder;
            EncoderItem *WheelEncoder;

            double EncRevTicks = 0;
            double WheelEncRevTicks = 0;

            double D_Time = 0;
    };
}

#endif