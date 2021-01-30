/****************************** Header ******************************\
Class Name: SwerveManager
File Name:	SwerveManager.h
Summary: Manager that holds the Swerve modules
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#ifndef SRC_SWERVE_MANAGER_H_
#define SRC_SWERVE_MANAGER_H_

#include "SwerveModule.h"
#include <iostream>

using namespace std;
using namespace frc;
using namespace Util;

namespace Components
{
    class SwerveManager : public OutputComponent
    {
        public:
            enum ModuleLoc{Front_Left = 0,Front_Right = 1,Back_Left = 2,Back_Right = 3};

            SwerveManager(string name, bool Wait, SwerveModule *FrontLeft, SwerveModule *FrontRight, SwerveModule *BackLeft, SwerveModule *BackRight);

            void Set(double rawV, double rawH, double rawS);
            void SetSwivel(double val);
            bool SetSwivelTarget(double Target);
            bool SetWheelTarget(double Target);
            bool SetTarget(double Wheel_Target, double Swivel_Target);

            void SetL(double L) {Length = L;};
            void SetW(double W) {Width = W;};

            void SetDelta(double D_Time);
            void ResetPID();
            void ResetSwivelEnc();
            void ResetWheelEnc();

            virtual void DeleteComponent() override;

            SwerveModule* Get(ModuleLoc Loc)
            {
                if (Loc == 0)
                {
                    return FL;
                }
                if (Loc == 1)
                {
                    return FR;
                }
                if (Loc == 2)
                {
                    return BL;
                }
                if (Loc == 3)
                {
                    return BR;
                }
            };

            virtual void Set(double val);
			virtual double Get();
            virtual void DefaultSet();
			virtual void Set(DoubleSolenoid::Value value);

            double GetMax(double Val1, double Val2) {return (Val1 > Val2 ? Val1 : Val2); };

        private:
            SwerveModule *FL;
            SwerveModule *FR;
            SwerveModule *BL;
            SwerveModule *BR;

            double Length = 0;
            double Width = 0;
            bool WaitSwivel = false;
    };
}

#endif