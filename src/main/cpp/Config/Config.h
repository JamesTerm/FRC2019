/****************************** Header ******************************\
Class Name: Config
File Name:	Config.h
Summary: 	Manages and loads the configuration file.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson, Chris Weeks
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com, chrisrweeks@aol.com
\********************************************************************/


#ifndef SRC_CONFIG_CONFIG_H_
#define SRC_CONFIG_CONFIG_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "ActiveCollection.h"
#include "../Systems/Drive.h"
#include "../Controls/ButtonControl.h"
#include "../Controls/AxisControl.h"
#include "../Controls/ToggleButtonControl.h"
#include "../Controls/GoalButtonControl.h"
#include "../Controls/SwerveControl.h"
#include "../Pugi/pugixml.h"
#include "../Goals/GoalSelector.h"
#include "../Util/Log.h"
#include "../Limelight/limelight.h"
#include "../Components/SparkMaxItem.h"
#include "../Components/REVColorSensorV3.h"
#include "backupConfig.h"

using namespace System;
using namespace pugi;
using namespace Lime;
namespace Configuration{

class Config {
public:
	Config(ActiveCollection *_activeCollection, Drive *_drive);
	//TODO: Make this a bool return
	void LoadValues(xml_document &doc);
	void AllocateComponents(xml_node &root);
	void AllocateDriverControls(xml_node &controls);
	void AllocateOperatorControls(xml_node &controls);
	virtual ~Config();
private:
	Joystick *m_driveJoy;
	Joystick *m_operatorJoy;
	ActiveCollection *m_activeCollection;
	Drive *m_drive;
	vector<string> getBindingStringList(string bindings);
	bool setBindingsToControl(vector<string> bindings, ControlItem *control);
	TeleOpGoal getTeleOpGoal(string goalString);
	SwerveModule::Location GetLocation(string Loc);
};

}
#endif /* SRC_CONFIG_CONFIG_H_ */
