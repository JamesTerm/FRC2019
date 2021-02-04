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
#include "../SparkMaxItem.h"
#include "../TalonSRXItem.h"

using namespace std;
using namespace frc;
using namespace Util;

namespace Components
{
    class SwerveModule : public OutputComponent
    {
        public:
            enum Location
            {
                Front_Left = 0,
                Front_Right = 1,
                Back_Left = 2,
                Back_Right = 3
            };

            SwerveModule(string name, Motor *SwivelMtr, Motor *WheelMtr, EncoderItem* SwivelEnc, EncoderItem* WheelEnc, double TicksPerRev, double WheelTicks);
            SwerveModule(string name, Motor *SwivelMtr, Motor *WheelMtr, double TicksPerRev, double WheelTicks);
            void SetLocation(Location Loc) {ModuleLoc = Loc;};
            void Set(double val, double SwivelVal);
            void SetSwivel(double SwivelVal);
            double GetSwivel();
            virtual void Set(double val);
			virtual double Get();

            double GetEnc();
            double GetSwivelEnc();
            double GetSwivelTarget() {return CurrentSwivelTarget;};
            double GetWheelTarget() {return CurrentWheelTarget;};
            void UpdateWheelRate();
            double GetWheelRate() {return LastSpeed;};
            void SetWheelSize(double Diameter) {WheelDi = Diameter;};

            void ResetEncs();
            void ResetWheelEnc();
            void ResetSwivelEnc();

            double GetWheelTicks(){return WheelEncRevTicks;};
            double GetSwivelTicks(){return EncRevTicks;};

            void ResetPID();
            void SetDeltaTime(double Time);
            void ProcessMotor(Motor *Subject, double Enc, PIDProfile *Profile, double Target, double TickRev);
            bool SetTargetWheel(double Target);
            bool SetTargetSwivel(double Target);
            bool SetTarget(double Wheel_Target, double Swivel_Target);
            bool SetSpeedTarget(double SPEEEEED);

            bool PointingIn(double Angle)
            {
                switch(ModuleLoc)
                {
                    case  Location::Front_Left:
                        return (Angle <= -90 || (Angle == 180));
                    break;
                    case  Location::Front_Right:
                        return (Angle >= 90 || (Angle == -180)); 
                    break;
                    case  Location::Back_Left:
                        return (Angle >= -90 && Angle <= 0);
                    break;
                    case  Location::Back_Right:
                        return (Angle >= 0 && Angle <= 90); 
                    break;
                }
                return false;
            };

            bool IsPointingIn()
            {
                return PointingIn(GetSwivelTarget());
            }

            virtual void DefaultSet();
			virtual void Set(DoubleSolenoid::Value value);
            virtual void DeleteComponent() override;

            Location GetLocation() {return ModuleLoc;};

            PIDProfile *WheelPID;
            PIDProfile *SwivelPID;
            PIDProfile *SpeedPID;

        private:
            enum InputType
            {
                EncoderType = 0,
                MotorType = 1
            };

            InputType GetType;

            Motor *Swivel;
            Motor *Wheel;
            EncoderItem *SwivelEncoder;
            EncoderItem *WheelEncoder;

            Location ModuleLoc = Location::Front_Left;

            double EncRevTicks = 0;
            double WheelEncRevTicks = 0;
            double CurrentSwivelTarget = 0;
            double CurrentWheelTarget = 0;
            double LastWheelTick = 0;
            double WheelDi = 0;
            double LastSpeed = 0;

            double D_Time = 0;
            double Dir = 1;
    };
}

#endif