/****************************** Header ******************************\
Class Name: NavX inherits AHRS, NativeComponent
File Name:	NavX.h
Summary: Provides methods to use and get values from sensors on the NavX board.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#ifndef SRC_COMPONENTS_NAVX_H_
#define SRC_COMPONENTS_NAVX_H_

#include <AHRS.h>

#include "NativeComponent.h"

namespace Components{
class NavX : public AHRS, public NativeComponent{
public:
	NavX() : AHRS(SerialPort::Port::kMXP, AHRS::SerialDataType::kProcessedData, 50), NativeComponent("NavX"){}
	NavX(bool Fake) : AHRS(SerialPort::Port::kMXP, AHRS::SerialDataType::kProcessedData, 50), NativeComponent("NavX"){FakeRun = true; SetUP();}
	NavX(SPI::Port spiPortId, uint8_t updateRateHz) : AHRS(spiPortId, updateRateHz), NativeComponent("NavX"){}
	NavX(SPI::Port spiPortId, int spiBitRate, uint8_t updateRateHz) : AHRS(spiPortId, spiBitRate, updateRateHz), NativeComponent("NavX"){}
	NavX(I2C::Port i2CPortId, uint8_t updateRateHz): AHRS(i2CPortId, updateRateHz), NativeComponent("NavX"){}
	NavX(SerialPort::Port serialPortId, SerialDataType dataType, uint8_t updateRateHz) : AHRS(serialPortId, dataType, updateRateHz), NativeComponent("NavX"){}
	NavX* GetRawComponent(){return this;}
	virtual ~NavX(){}
	double GetNavXAngle(){
		if (FakeRun)
		{
			return OutputTable->GetNumber("NavX-Y", 0);
		}
		else
		{
			double angle = GetAngle();
			return angle;
		}
	}

	double GetConstAngle()
	{
		return GetNavXAngle() - RefAngle;
	}

	void ResetNav()
	{
		RefAngle = GetConstAngle();
		Reset();
		ResetFakeNav();
	}
	
	void ResetFakeNav()
	{
		nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutBoolean("NavX-Reset", true);
	}
	virtual void DeleteComponent() {delete this;};
private:
	bool FakeRun = false;
	double RefAngle = 0;
	void SetUP()
	{
		nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutNumber("NavX-Y", 0);
		nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutNumber("NavX-X", 0);
		nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutNumber("NavX-Z", 0);
		nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutBoolean("NavX-Reset", true);

		nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutNumber("Dis-Y", 0);
		nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutNumber("Dis-X", 0);
		nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutNumber("Dis-Z", 0);
		nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutBoolean("Dis-Reset", true);
	}
};
	

}
	

#endif /* SRC_COMPONENTS_NAVX_H_ */
