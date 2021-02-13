/****************************** Header ******************************\
Class Name: NativeComponent
File Name:	OutputComponent.h
Summary: Abstraction for all programmable robot components.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#ifndef SRC_COMPONENTS_NATIVECOMPONENT_H_
#define SRC_COMPONENTS_NATIVECOMPONENT_H_

#include <string>
#include <frc/WPILib.h>

#include "../Global.h"

#include <networktables/NetworkTableInstance.h>

using namespace frc;
using namespace std;

namespace Components
{
	class NativeComponent
	{
		public:
			string name;
			virtual void DeleteComponent() = 0;
			void FromTable(bool Yes) { UseTable = !Yes; }
			virtual void UpdateComponent() {};
			NativeComponent(){}
			NativeComponent(string _name){ name = _name; }
			virtual ~NativeComponent(){}
			std::shared_ptr<NetworkTable> OutputTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
		protected:
			bool UseTable = false;
	};

	
    struct double_Vector2
    {
        double X = 0;
        double Y = 0;

		double LastX = 0;
		double LastY = 0;
    };
}

#endif /* SRC_COMPONENTS_NATIVECOMPONENT_H_ */
