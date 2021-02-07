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
#include "../NavX.h"
#include <iostream>

using namespace std;
using namespace frc;
using namespace Util;

namespace Components
{
    class SwerveManager : public OutputComponent
    {
        public:

            SwerveManager(string name, bool Wait, SwerveModule *FrontLeft, SwerveModule *FrontRight, SwerveModule *BackLeft, SwerveModule *BackRight, double _Length, double _Width);
            SwerveManager(string name, bool Wait, SwerveModule *FrontLeft, SwerveModule *FrontRight, SwerveModule *BackLeft, SwerveModule *BackRight, NavX *Nav, double _Length, double _Width);
            SwerveManager(string name, bool Wait, vector<SwerveModule*> Swerve_Modules, NavX *Nav, double _Length, double _Width);

            void Set(double rawV, double rawH, double rawS);
            void SetSwivel(double val);
            bool SetSwivelTarget(double Target);
            bool SetWheelTarget(double Target);
            bool SetTarget(double Wheel_Target, double Swivel_Target);

            bool SetSwivelTargetAt(SwerveModule::Location Loc, double Target);
            bool SetWheelTargetAt(SwerveModule::Location Loc, double Target);
            void SetSwivelAt(SwerveModule::Location Loc, double Power);
            void SetWheelAt(SwerveModule::Location Loc, double Power);

            void SetWheelDiameter(double Diameter);

            void SetL(double L) {Length = L;};
            void SetW(double W) {Width = W;};

            void SetDelta(double D_Time);
            void ResetPID();
            void ResetSwivelEnc();
            void ResetWheelEnc();

            void UpdateLoc();
            void UpdateModules();

            void UpdateSystem(double D_Time);

            virtual void DeleteComponent() override;

            SwerveModule* Get(SwerveModule::Location Loc)
            {
                for(int i = 0; i < Modules.size(); i++)
                {
                    if(Modules.at(i)->GetLocation() == Loc)
                    {
                        return Modules.at(i);
                    }
                }
                return nullptr;
            };

            vector<SwerveModule*> GetModules();

            virtual void Set(double val);
			virtual double Get();
            virtual void DefaultSet();
			virtual void Set(DoubleSolenoid::Value value);
            void SetMaxPow(double Max)
            {
                MaxValParam = Max;
            }
            double_Vector2* GetBotPos() {return Pos;};

            double GetMax(double Val1, double Val2) {return (Val1 > Val2 ? Val1 : Val2); };

            double GetEnc()
            {
                double Average = 0;
                for(int i = 0; i < Modules.size(); i++)
                {
                    Average += Modules.at(i)->GetEnc();
                }
                return Average / Modules.size();
            };

        private:
            vector<SwerveModule*> Modules;
            
            NavX *RobotNav = nullptr;

            double Length = 0;
            double Width = 0;
            double WheelAngle = 0;
            double_Vector2 *Pos;
            double Del_Time = 0;
            double MaxValParam = 1;
            bool WaitSwivel = false;
    };
}

#endif