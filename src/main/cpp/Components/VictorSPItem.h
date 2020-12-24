/****************************** Header ******************************\
Class Name: VictorSPItem inherits OutputComponent
File Name:	VictorSPItem.h
Summary: Abstraction for the WPIlib VictorSP that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#ifndef SRC_COMPONENTS_VICTORSPITEM_H_
#define SRC_COMPONENTS_VICTORSPITEM_H_

#include "Motor.h"

namespace Components
{
	class VictorSPItem final : public Motor
	{
		public:
			VictorSPItem(){}
			virtual ~VictorSPItem(){}
			VictorSPItem(string name, int channel, bool reversed) : Motor(name)
			{
				victor = new VictorSP(channel);
				Reversed = reversed;
			}

			int GetPolarity();
			string GetName();
			void Stop();
			virtual void Set(double val) override;
			virtual double Get() override;
			//for reference
			//operator VictorSP *() { return victor;}
			VictorSP *AsVictorSP() { return victor; }
		private:
			VictorSP *victor;
			bool Reversed;
			virtual void DefaultSet() override;
			virtual void Set(DoubleSolenoid::Value value) override;
	};
}

#endif /* SRC_COMPONENTS_VICTORSPITEM_H_ */
