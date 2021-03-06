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
#include "../Global.h"
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
		m_activeCollection->Add(new NavX(Fake));
		Log::General("NavX detected");
	}
	else
		Log::General("Failed to load the NavX: Disabling NavX by default");
	if (useLimelight)
	{
    	limelight* lime = new limelight(Fake);
		m_activeCollection->Add(lime);
		Log::General("Added Limelight");
	}
	else{
		Log::General("Limelight not added");
	}
	m_driveJoy = new Joystick(DriverSlot);
	m_operatorJoy = new Joystick(OperatorSlot);
	
	backupConfig::SetLoopTime(0.01);
}

void backupConfig::SetComponents()
{
	#ifdef __Use_RobotBase__
	backupConfig::AddPIDProfile("SwivelPIDControl", 0.8, 0.01, 0, 1, 1000);
	backupConfig::AddPIDProfile("CoverPIDControl", 0.8, 0.01, 0, 1, 1000);
	backupConfig::AddPIDProfile("AutoPositionX", 9, 7, 0, 0.15, 10);
	backupConfig::AddPIDProfile("AutoPositionY", 9, 7, 0, 0.15, 10);
	#else
	backupConfig::AddPIDProfile("SwivelPIDControl", 5, 0, 0, 1, 100);
	backupConfig::AddPIDProfile("CoverPIDControl", 0.8, 0.01, 0, 1, 1000);
	backupConfig::AddPIDProfile("AutoPositionX", 9, 7, 0, 0.15, 10);
	backupConfig::AddPIDProfile("AutoPositionY", 9, 7, 0, 0.15, 10);
	#endif

	backupConfig::AddSparkMax("WheelFL", 3, false);
	backupConfig::AddSparkMax("WheelFR", 4, false);
	backupConfig::AddSparkMax("WheelBL", 5, false);
	backupConfig::AddSparkMax("WheelBR", 6, false);

	backupConfig::AddTalonSRX("SwivelFL", 7, false, true);
	backupConfig::AddTalonSRX("SwivelFR", 8, false, true);
	backupConfig::AddTalonSRX("SwivelBL", 9, false, true);
	backupConfig::AddTalonSRX("SwivelBR", 10, false, true);

	backupConfig::Link_Motor_PositionProfile("SwivelFL", "SwivelPIDControl");
	backupConfig::Link_Motor_PositionProfile("SwivelFR", "SwivelPIDControl");
	backupConfig::Link_Motor_PositionProfile("SwivelBL", "SwivelPIDControl");
	backupConfig::Link_Motor_PositionProfile("SwivelBR", "SwivelPIDControl");

	backupConfig::AddSwerveModule("FL", "SwivelFL", "WheelFL", 4096, 4096, SwerveModule::Location::Front_Left);
	backupConfig::AddSwerveModule("FR", "SwivelFR", "WheelFR", 4096, 4096, SwerveModule::Location::Front_Right);
	backupConfig::AddSwerveModule("BL", "SwivelBL", "WheelBL", 4096, 4096, SwerveModule::Location::Back_Left);
	backupConfig::AddSwerveModule("BR", "SwivelBR", "WheelBR", 4096, 4096, SwerveModule::Location::Back_Right);

	backupConfig::AddSwerveManager("SwerveDT", false, 0.8, "FL FR BL BR", m_activeCollection->GetNavX(), 3, 3);

	backupConfig::AddVictorSPX("Intake", 11, false);
	backupConfig::AddVictorSPX("Revolver", 12, false);
	backupConfig::AddVictorSPX("Shooter", 13, false);
	backupConfig::AddVictorSPX("Cover", 14, false);

	backupConfig::AddVictorSPX("PullUp-Left", 15, false);
	backupConfig::AddVictorSPX("PullUp-Right", 16, false);

	backupConfig::AddEncoder("RevolverEncoder", 1, 2, false);
	backupConfig::AddEncoder("CoverEncoder", 3, 4, false);
	backupConfig::AddEncoder("ShooterEncoder", 5, 6, false);

	backupConfig::Link_Motor_Encoder("Cover", "CoverEncoder");
	backupConfig::Link_Motor_PositionProfile("Cover", "CoverPIDControl");

	backupConfig::AddDoubleSolenoid("IntakePiston", 1, 2, DoubleSolenoid::Value::kReverse, false);
}

void backupConfig::SetControls()
{
	backupConfig::AddSwerveControl("SwerveControl", SwerveControl::DriveCalculation::Robot_Oriented, 0, 1, 4, 0.08, 0.8, false, "SwerveDT", backupConfig::JoystickControler::Driver);

	backupConfig::AddAxisControl("IntakeControl", "Intake", 1, 0.8, false, false, false, false, 0.01, backupConfig::JoystickControler::Operator);
	backupConfig::AddAxisControl("RevolverControl", "Revolver", 0, 0.8, false, false, false, false, 0.01, backupConfig::JoystickControler::Operator);

	backupConfig::AddAxisControl("ShooterControl", "PullUp-Left, PullUp-Right", 2, 0.5, false, false, false, false, 0.01, backupConfig::JoystickControler::Operator);
	
	backupConfig::AddToggleControl("IntakePistonControl", "IntakePiston", 1, 1, false, false, backupConfig::JoystickControler::Operator);

	backupConfig::AddGoalAxisControl("CoverControl", TeleOpGoal::eMotorPosition, vector<int> {2}, vector<string> {"Cover", "CoverEncoder"}, 1, vector<int> {}, 0, false, true, backupConfig::JoystickControler::Operator);

	backupConfig::SetAutoPath("Bounce");
}

backupConfig::~backupConfig(){}