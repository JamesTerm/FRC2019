/****************************** Header ******************************\
Class Name: backupConfig
File Name:	backupConfig.cpp
Summary: 	If the config file doesnt exist config loads this instead
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#include "backupConfig.h"
#include <string.h>
#include <stdio.h>

using namespace std;
using namespace System;
using namespace Configuration;
using namespace pugi;
using namespace frc;

backupConfig::backupConfig(ActiveCollection *_activeCollection, Drive *_drive)
{
    m_activeCollection = _activeCollection;
	m_drive = _drive;
    SetInfo();
    SetComponents();
    SetControls();
}

void backupConfig::SetInfo()
{
    Log::atComp = AtCompVal;
    Log::General("Backup Config Version: " + to_string(Version), true);

	if(useNavX){
		m_activeCollection->Add(new NavX());
		Log::General("NavX detected");
	}
	else
		Log::General("Failed to load the NavX: Disabling NavX by default");
	if (useLimelight)
	{
    	limelight* lime = new limelight();
		m_activeCollection->Add(lime);
		Log::General("Added Limelight");
	}
	else{
		Log::General("Limelight not added");
	}
	m_driveJoy = new Joystick(DriverSlot);
	m_operatorJoy = new Joystick(OperatorSlot);
}

void backupConfig::SetComponents()
{
	backupConfig::AddVictor("left0", 2, false);
	backupConfig::AddVictor("left1", 3, false);
	backupConfig::AddVictor("right0", 4, false);
	backupConfig::AddVictor("right1", 1, false);
}

void backupConfig::SetControls()
{
	backupConfig::AddAxisControl("leftDrive", "left0,left1", 1, 0.5, false, false, false, false, 0.08, backupConfig::JoystickControler::Driver);
	backupConfig::AddAxisControl("rightDrive", "right0,right1", 1, 0.5, false, false, false, false, 0.08, backupConfig::JoystickControler::Driver);
}

backupConfig::~backupConfig(){}