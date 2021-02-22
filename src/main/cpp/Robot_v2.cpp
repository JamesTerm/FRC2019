#include "Robot.h"
#ifndef __Use_RobotBase_Depreciated__

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <string>
//#include <frc/WPILib.h>
//#include <frc/RobotBase.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>

#include "Config/Config.h"
#include "Config/ActiveCollection.h"
#include "Systems/Drive.h"
#include "Controls/AxisControl.h"
#include "Autonomi/Autons.h"
#include "Util/Log.h"
#include "Goals/FRC2019_Goals.h"
#include "Goals/GoalSelector.h"
#include "Util/VisionTarget.h"
#include "Limelight/limelight.h"
#include "Util/UtilityFunctions.h"
#include "Util/LinePaths.h"

#include "Util/RobotStatus.h"
#include "Util/FrameworkCommunication.h"
//Simulation includes
#include "Simulation/Modules/Robot/SwerveRobot/Simulator_Interface.h"

using namespace frc;
using namespace System;
using namespace Controls;
using namespace Configuration;
using namespace Autonomi;
using namespace Logger;
using namespace Lime;

#define VERSION 1    //!< Defines the program version for the entire program.
#define REVISION "A" //!< Defines the revision of this version of the program.


class Robot : public frc::TimedRobot 
{
private:
    #pragma region _members_
    Drive *m_drive;
    ActiveCollection *m_activeCollection=nullptr; //!< Pointer to the only instantiation of the ActiveCollection Class in the program
    const string m_driveStraight = "drive";
    MultitaskGoal* m_masterGoal;
    MultitaskGoal* m_teleOpMasterGoal;
    std::shared_ptr<NetworkTable> AutoTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
    nt::NetworkTableInstance m_inst; //!Network tables
    shared_ptr<NetworkTable> m_dashboardTable;
    string m_autonOptions[5] = {"DriveStraight","OnePieceAuto","TwoPieceAuto","DEBUG","NONE"};
    string m_positionOptions[5] = {"Level 1 Left", "Level 1 Center", "Level 1 Right", "Level 2 Left", "Level 2 Right"};
    std::atomic<bool> m_exit = false;
   	frc::Timer m_Timer; //use frc timer to take advantage of stepping in simulation (works fine for actual roboRIO too)
	double m_LastTime=0.0;  //used for time slices
	double m_simLastTime=0.0;  //for simulation time slices
    Module::Robot::Simulator_Interface m_simulation;
    bool m_IsConfigLoaded=false;
    #pragma endregion
    void LoadConfig()
    {
        //This only needs to happen one time!
        if (!m_IsConfigLoaded)
        {
            m_IsConfigLoaded=true;
            Config *config = new Config(m_activeCollection, m_drive); //!< Pointer to the configuration file of the robot
        }
    }
    void TimeSlice()
    {
        
        const double CurrentTime = m_Timer.GetFPGATimestamp();
        #if 1
        const double DeltaTime = CurrentTime - m_LastTime;
        #else
        const double DeltaTime=0.01;  //It's best to use synthetic time for simulation to step through code
        #endif
        m_LastTime = CurrentTime;
        //sanity check
        //frc::SmartDashboard::PutNumber("time_delta",DeltaTime);
        m_drive->Update(DeltaTime);
    }
    void SimulatorTimeSlice()
    {
        const double CurrentTime = m_Timer.GetFPGATimestamp();
        #if 1
        const double DeltaTime = CurrentTime - m_simLastTime;
        #else
        const double DeltaTime=0.01;
        #endif
        m_simLastTime = CurrentTime;
        //sanity check
        //frc::SmartDashboard::PutNumber("time_delta",DeltaTime);
        m_simulation.TimeSlice(DeltaTime);
    }

 public:
    //defaults to 20ms... really want 10
    Robot() : TimedRobot(10_ms) 
    {
       	m_activeCollection = new ActiveCollection();
        m_drive = new Drive(m_activeCollection);
        //extend the life of the active region's copy by adding a reference to this variable
        //TODO: Events
    }
    ~Robot()
    {
        delete m_drive;
        m_drive = nullptr;
        delete m_activeCollection;
        m_activeCollection = nullptr;
    }
    void RobotInit() override
    {
        FrameworkCommunication::GetInstance();
        nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutBoolean("RUN_ROBOT", false);
        Log::restartfile();
        Robot::LoadConfig();
        Log::General("Program Version: " + to_string(VERSION) + " Revision: " + REVISION, true);
        SmartDashboard::init(); //!< Must have this for smartdashboard to work properly
        m_inst = nt::NetworkTableInstance::GetDefault(); //!< Network tables
        m_dashboardTable = m_inst.GetTable("DASHBOARD_TABLE");
        m_dashboardTable->PutStringArray("AUTON_OPTIONS", m_autonOptions);
        m_dashboardTable->PutStringArray("POSITION_OPTIONS", m_positionOptions);
        // Util::FrameworkCommunication::GetInstance().SendData("MESSAGE","yeetus");//? Temporary
        //Give active collection to simultion to access motors and encoders:
        m_simulation.ActiveCollection_Init(m_activeCollection);
    }
    void RobotPeriodic() override
    {}
    void AutonomousInit() override
    {}
    void AutonomousPeriodic() override
    {}
    void TeleopInit() override
    {
       	Robot::LoadConfig();
        nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutBoolean("RUN_ROBOT", true);
        Util::RobotStatus::GetInstance().NotifyState(Util::RobotState::Teleop);
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
        if (m_activeCollection->Get("LimeLight") != nullptr)
        limelight* lime = (limelight*)(m_activeCollection->Get("LimeLight"));
        if (m_activeCollection->GetNavX() != nullptr)
        m_activeCollection->GetNavX()->ResetNav();
    }
    void TeleopPeriodic() override
    {
        //Note: I may want to change how this works for this code, unlike example code which uses the same
        //code path for teleop and autonomous... for now it is like the example code
        TimeSlice();
    }
    void DisabledInit() override
    {}
    void DisabledPeriodic() override
    {
       	nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutBoolean("RUN_ROBOT", false);
        Util::RobotStatus::GetInstance().NotifyState(Util::RobotState::Disabled);
    }
    void TestInit() override
    {}
    void TestPeriodic() override
    {}
    void SimulationInit () override
    {
        m_simulation.SimulationInit();
    }
    void SimulationPeriodic () override
    {
        SimulatorTimeSlice();
    }
};

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif

#endif
