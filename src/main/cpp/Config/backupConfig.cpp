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
		m_activeCollection->Add(new NavX(true));
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
	backupConfig::AddSparkMax("WheelFL", 3, false);
	backupConfig::AddSparkMax("WheelFR", 4, false);
	backupConfig::AddSparkMax("WheelBL", 5, false);
	backupConfig::AddSparkMax("WheelBR", 6, false);

	backupConfig::AddTalonSRX("SwivelFL", 7, false, true);
	backupConfig::AddTalonSRX("SwivelFR", 8, false, true);
	backupConfig::AddTalonSRX("SwivelBL", 9, false, true);
	backupConfig::AddTalonSRX("SwivelBR", 10, false, true);

	backupConfig::AddSwerveModule("FL", "SwivelFL", "WheelFL", 4096, 24);
	backupConfig::AddSwerveModule("FR", "SwivelFR", "WheelFR", 4096, 24);
	backupConfig::AddSwerveModule("BL", "SwivelBL", "WheelBL", 4096, 24);
	backupConfig::AddSwerveModule("BR", "SwivelBR", "WheelBR", 4096, 24);

	backupConfig::AddSwerveManager("SwerveDT", true, 0.8, "FL", "FR", "BL", "BR");
}

void backupConfig::SetControls()
{
	backupConfig::AddSwerveControl("SwerveControl", SwerveControl::DriveCalculation::Warthog, 0, 1, 4, 0.08, 0.8, false, 10, 10, "SwerveDT", backupConfig::JoystickControler::Driver);
}

backupConfig::~backupConfig(){}