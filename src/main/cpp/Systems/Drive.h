/****************************** Header ******************************\
Class Name:	Drive [final]
File Name: 	Drive.h
Summary: 	Drive loop
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Ryan Cooper, Dylan Watson
Email:	cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com
\*********************************************************************/

#pragma once

#ifndef SYSTEMS_DRIVE_H_
#define SYSTEMS_DRIVE_H_

#include <frc/WPILib.h>
#include "../util/LoopChecks.h"
#include "../Controls/ControlItem.h"
#include "../Goals/FRC2019_Goals.h"

using namespace frc;
using namespace std;
using namespace Util;
using namespace Controls;

namespace System
{
    class Drive final
    {
        public:
	        Drive();
			Drive(ActiveCollection* activeCollection);
	        void Update(double deltaTime);
	        void AddControlDrive(ControlItem *control);
	        void AddControlOperate(ControlItem *control);
			void SetUseDrive(bool enable) { m_DisableDrive = !enable; }
			void SetUseOperator(bool enable) { m_DisableOperator = !enable; }
	private:
			bool m_DisableDrive=false;
			bool m_DisableOperator=false;
			MultitaskGoal* m_teleOpGoal;
			ActiveCollection* m_activeCollection;
			vector<ControlItem*> m_driveControlCollection;
			vector<ControlItem*> m_operateControlCollection;
	};
} /* namespace Systems */

#endif /* SYSTEMS_DRIVE_H_ */
