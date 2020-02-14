/****************************** Header ******************************\
Class Name: REVColorSensorV3 inherits OutputComponent
File Name:	REVColorSensorV3.h
Summary: Abstraction for all programmable robot output components.
Project:     BroncBotzFRC2020
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Emily Martinez, Guadalupe Rodriguez, Shruti Venkatramanan
Email: shruti.venkat05@gmail.com
\********************************************************************/


#ifndef SRC_COMPONENTS_REVColorSensorV3_H_
#define SRC_COMPONENTS_REVColorSensorV3_H_

#include <rev/REVColorSensorV3.h>

#include "OutputComponent.h"

using namespace std;
using namespace frc;
using namespace rev;
namespace Components{
class REVColorSensorV3: public OutputComponent{
private:
    REVColorSensorV3 *REV;
	int channel;
	bool reversed;
	string Name;
	double Offset;

public:
	REVColor();
	REVColor(int _channel, string _name, bool _reversed);
	double GetColor();
    
	double GetRawColor();
	uint32_t GetProximity();
	void ConfigureColorSensor();
	virtual void Set(double val) override;
	string GetName();
	void (frc::I2C::Port port);
	virtual void DefaultSet() override;
	virtual ~REVColor();
	REVColorSensorV3 *REVColor() { return REV;}
    void ConfigureProximitySensor(); 
	
	
};
}
3
#endif /* SRC_COMPONENTS_REVColorSensorV3_H_ */