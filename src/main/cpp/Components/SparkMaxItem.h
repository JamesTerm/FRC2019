/****************************** Header ******************************\
Class Name: Spark inherits OutputComponent
File Name:	SparkMaxItem.h
Summary: Abstraction for the Spark Max controller
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll, Shruti Venkatramanan, Guadalupe Rodriguez, Emily Martinez
Email: irobot983@gmail.com
\********************************************************************/

#ifndef SRC_COMPONENTS_SPARKMAXITEM_H_
#define SRC_COMPONENTS_SPARKMAXITEM_H_

#include <rev/CANSparkMax.h>
#include "Motor.h"

using namespace std;
using namespace frc;
using namespace rev;
namespace Components{
class SparkMaxItem : public Motor{
private:
	CANSparkMax *Max;
	int channel;
	bool reversed;
	string Name;
	double Offset;

public:
	SparkMaxItem();
	SparkMaxItem(int _channel, string _name, bool _reversed);
	double GetEncoderValue();
    
	virtual double Get() ;
	int GetPolarity();
	void Reset();
	virtual void Set(double val) ;
	virtual void Set(DoubleSolenoid::Value value) ;
	virtual void Stop() ;
	string GetName();
	virtual void DefaultSet() ;
	virtual ~SparkMaxItem();
	CANSparkMax *AsSparkMax() { return Max;}
    void ResetEncoderValue(); 
	
	
};
}

#endif /* SRC_COMPONENTS_SPARKMAXITEM_H_ */