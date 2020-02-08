/****************************** Header ******************************\
Class Name: Robot inherits SampleRobot
File Name: Robot.cpp
Summary: Entry point from WPIlib, main class where robot routines are
started.
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson, Chris Weeks
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com, 
chrisrweeks@aol.com
\********************************************************************/

#include <iostream>
#include "Robot.h"
using namespace std;

/**
 * Constructor
 */
Robot::Robot() 
{
	m_activeCollection = new ActiveCollection();
	m_drive = new Drive(m_activeCollection);
	//extend the life of the active region's copy by adding a reference to this variable
	//TODO: Events
}

Robot::~Robot()
{
	delete m_drive;
	m_drive = nullptr;
	delete m_activeCollection;
	m_activeCollection = nullptr;
}

/*
 * Initialization method of the robot
 * This runs right after Robot() when the code is started
 * Creates Config
 */
void Robot::RobotInit()
{
	Log::restartfile();
	Log::General("Program Version: " + to_string(VERSION) + " Revision: " + REVISION, true);
	Config *config = new Config(m_activeCollection, m_drive); //!< Pointer to the configuration file of the robot
	SmartDashboard::init(); //!< Must have this for smartdashboard to work properly
	m_inst = nt::NetworkTableInstance::GetDefault(); //!< Network tables
	m_dashboardTable = m_inst.GetTable("DASHBOARD_TABLE");
	m_dashboardTable->PutStringArray("AUTON_OPTIONS", m_autonOptions);
	m_dashboardTable->PutStringArray("POSITION_OPTIONS", m_positionOptions);
}

/*
 * Method that runs at the beginning of the autonomous mode
 * Uses the SmartDashboard to select the proper Autonomous mode to run
 */
void Robot::Autonomous()
{
	
	m_masterGoal = new MultitaskGoal(m_activeCollection, false);

	Log::General("Autonomous Started");
	//TODO: Make defaults set now and call the active collection
	m_activeCollection->DoubleSolenoidDefault();
#ifndef _Win32
	string autoSelected = m_dashboardTable->GetString("AUTON_SELECTION", m_driveStraight);
	string positionSelected = m_dashboardTable->GetString("POSITION_SELECTION", "NONE"); // Default auto is drive striaght 
	if (!SelectAuton(m_activeCollection, m_masterGoal, autoSelected, positionSelected)) // Selection
	{
		m_dashboardTable->PutString("AUTON_FOUND", "UNDEFINED AUTON OR POSITION SELECTED");
	}
#endif
	m_masterGoal->AddGoal(new Goal_TimeOut(m_activeCollection, 15.0));
	m_masterGoal->AddGoal(new Goal_ControllerOverride(m_activeCollection)); //!< This is for FRC 2019 SANDSTORM! Be aware that if Sandstorm is removed, this NEEDS to be removed.
	//TODO: Make the auto configurable (turn on/turn off) OR add a no auton feature to the dashboard
	m_masterGoal->Activate();
	m_activeCollection->SetActiveGoal(m_masterGoal);
	double dTime = 0.010;
	double current_time = 0.0;
	while (m_masterGoal->GetStatus() == Goal::eActive && _IsAutononomous() && !IsDisabled())
	{
		m_drive->Update(dTime); //!< Check for controller input for controller override -> will be removed if sandstorm is removed
		m_masterGoal->Process(dTime); //!< Process the autonomous goal
		current_time += dTime;
#ifdef _Win32
		SmartDashboard::PutNumber("DELTA TIME", current_time);
		SmartDashboard::PutString("AUTONOMOUS", "RUNNING");
#endif		
		Wait(dTime);
	}
	/* THIS LOOP WILL HAVE TO BE REMOVED IF SANDSTORM IS REMOVED */
	while(_IsAutononomous() && !IsDisabled()) //!< Loop to run manual control once the auto function is done in sandstorm
	{
#ifdef _Win32
		SmartDashboard::PutString("AUTONOMOUS", "OVERRIDEN!");
#endif
		m_drive->Update(dTime); //!< Run robot teleOp
		Wait(dTime);
	}
	m_masterGoal->~MultitaskGoal(); //!< Destroy the masterGoal in order to prevent multiple ControllerOverrides running at once
}

/*
 * Called when teleop starts
 */
//test method doesnt work in rio for some reason...
//TODO: Potentially make a "test" tag in the config that can toggle this?
void Robot::Teleop()
{
	m_activeCollection->GetActiveGoal()->~MultitaskGoal(); //!< Destroy any pre-existing masterGoal that was not properly disposed of
	m_teleOpMasterGoal = new MultitaskGoal(m_activeCollection, false);
	//m_teleOpMasterGoal->AddGoal(new Goal_TimeOut(m_activeCollection, 15));
	//m_teleOpMasterGoal->AddGoal(new Goal_ControllerOverride(m_activeCollection));
	m_activeCollection->SetActiveGoal(m_teleOpMasterGoal);
	// m_activeCollection->GetActiveGoal()->AddGoal(new Goal_TimeOut(m_activeCollection, 3000));
	// m_activeCollection->GetActiveGoal()->Activate();
	//TODO: Talk to Ian about this
	Log::restartfile();
	Log::General("Teleoperation Started.");
	double LastTime = GetTime();
	//We can test teleop auton goals here a bit later
	//PotentiometerItem* pot = (PotentiometerItem*)m_activeCollection->Get("pot");
	limelight* lime = (limelight*)(m_activeCollection->Get("LimeLight"));

	while (IsOperatorControl() && !IsDisabled())
	{
		const double CurrentTime = GetTime();
		#ifndef _Win32
		const double DeltaTime = CurrentTime - LastTime;
		#else
		const double DeltaTime=0.01;  //It's best to use sythetic time for simulation to step through code
		#endif
		LastTime = CurrentTime;
		if (DeltaTime == 0.0) continue;  //never send 0 time
		m_drive->Update(DeltaTime);
		Wait(0.010); 
	}
}

/*
 * Called when the Test period starts
 */
void Robot::Test()
{
	cout << "entered" << endl;

	m_activeCollection->GetActiveGoal()->~Goal();
	cout << "set active goal" << endl;

	m_Drive = new Goal_MoveForward(m_activeCollection, 5, 0.6, 5);
	Log::General("Made Goal",true);
	
	m_Drive->Activate();
	Log::General("Activated",true);

	while(m_Drive->GetStatus() == Goal::eActive && !IsDisabled())
	{
	Log::General("Activated",true);
	
		m_Drive->Process(0.01);
		Wait(0.01);
	}
	

/*
	double LastTime = GetTime();
	int lastPos = 0;
	((TalonSRXItem*)m_activeCollection->Get("shooter0"))->SetQuadraturePosition(0);
	
	
	while(!IsDisabled()){
		const double CurrentTime = GetTime();
		const double DeltaTime = CurrentTime - LastTime;

		int currentPos = ((TalonSRXItem*)m_activeCollection->Get("shooter0"))->GetQuadraturePosition();
		int deltaPos = abs(currentPos - lastPos);
		double velocity = deltaPos / DeltaTime;

		LastTime = CurrentTime;
		if (DeltaTime == 0.0) continue;  //never send 0 time
		m_drive->Update(DeltaTime);
		Wait(0.010);
	}*/
}

void Robot::StartCompetition() {
  auto& lw = *frc::LiveWindow::GetInstance();

  RobotInit();

  // Tell the DS that the robot is ready to be enabled
  HAL_ObserveUserProgramStarting();

  while (!m_exit) {
    if (IsDisabled()) {
      m_ds.InDisabled(true);
      Disabled();
      m_ds.InDisabled(false);
      while (IsDisabled()) m_ds.WaitForData();
    } else if (IsAutonomous()) {
      m_ds.InAutonomous(true);
      Autonomous();
      m_ds.InAutonomous(false);
      while (IsAutonomous() && IsEnabled()) m_ds.WaitForData();
    } else if (IsTest()) {
      lw.SetEnabled(true);
      frc::Shuffleboard::EnableActuatorWidgets();
      m_ds.InTest(true);
      Test();
      m_ds.InTest(false);
      while (IsTest() && IsEnabled()) m_ds.WaitForData();
      lw.SetEnabled(false);
      frc::Shuffleboard::DisableActuatorWidgets();
    } else {
      m_ds.InOperatorControl(true);
      Teleop();
      m_ds.InOperatorControl(false);
      while (IsOperatorControl() && IsEnabled()) m_ds.WaitForData();
    }
  }
}

void Robot::EndCompetition() { m_exit = true; }

void Robot::Disabled() { /*Do nothing for now*/ }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }  //!< This identifies Robot as the main Robot starting class
#endif