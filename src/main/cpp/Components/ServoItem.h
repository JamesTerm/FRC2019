/****************************** Header ******************************\
Class Name: ServoItem inherits OutputComponent
File Name:	ServoItem.h
Summary: Servo controller
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot983@gmail.com
\********************************************************************/

#ifndef SRC_COMPONENTS_SERVOITEM_H_
#define SRC_COMPONENTS_SERVOITEM_H_
#include "OutputComponent.h"
#include <frc/Servo.h>

using namespace std;
using namespace frc;

namespace Components
{
    class ServoItem : public OutputComponent
    {
        public:
            enum ServoType
            {
                Continuous = 0,
                Limited = 1
            };

            ServoItem(string name, int port, double MaxAngle, ServoType Type, bool Real);
            virtual void Set(double val) override;
            void AngleSet(double angle);
            void SetOffline();
			virtual double Get() override;
            ServoType GetType() {return ServoCal;};
            virtual void ResetData(){};
            virtual double GetData(){return CurrentPercent;};
			
			virtual void DefaultSet() override;
			virtual void Set(DoubleSolenoid::Value value) override;
            virtual void DeleteComponent() override;
            void UpdateComponent() override;
			virtual ~ServoItem(){};
        
        private:
            Servo* BotServo = nullptr;
            double CurrentPercent = 0;
            double Angle = 0;
            ServoType ServoCal;
    };
}

#endif /* SRC_COMPONENTS_SERVOITEM_H_ */