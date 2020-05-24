/****************************** Header ******************************\
Class Name: (multiple classes)
File Name: FRC2019_Goals.h
Summary: All implemented Goals go here. Region statements categorize
types
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Chris Weeks
Email: chrisrweeks@aol.com
\********************************************************************/
#pragma once

#include <iostream>
#include "cmath"
#include "Goal.h"
#include "../Util/VisionTarget.h"
#include "../Util/Units/Distances.h"
#include "../Util/FrameworkCommunication.h"
#include "../Components/REVColorSensorV3.h"
#include "../Util/LinePaths.h"
#include "../Limelight/limelight.h"

using namespace Components;
using namespace std;
using namespace Lime;

//?HINT: ctrl+k then ctrl+0 will collapse all regions
//?HINT: ctrl+k then ctrl+[ will collapse all regions within the current scope of the cursor
//?HINT: ctrl+k then ctrl+] will uncollapse all regions within the current scope of the cursor
//?HINT: ctrl+k then ctrl+j will uncollapse all regions

//Atomic Goals go in this region
#pragma region AtomicGoals

//Time based goals go in this region
#pragma region TimerGoals
/* Goal_Wait
    * This goal will wait a specified amount of time then return eCompleted. Use it for delay features.
    ! NOTE: This goal is different then Goal_TimeOut
    */
class Goal_Wait_ac : public AtomicGoal
{
public:
  Goal_Wait_ac(ActiveCollection *activeCollection, double timeOut)
  {
    m_Status = eInactive;
    m_activeCollection = activeCollection;
    m_currentTime = 0;
    m_timeOut = timeOut;
  }
  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();

protected:
  ActiveCollection *m_activeCollection;
  double m_currentTime;
  double m_timeOut;
};
/* Goal_TimeOut
     * This goal will wait a specified amount of time then return eFailed. Use to time out a goal if it takes too long
     ! NOTE: This goal is different than Goal_Wait
     */
class Goal_TimeOut : public Goal_Wait_ac
{
public:
  Goal_TimeOut(ActiveCollection *activeCollection, double timeOut) : Goal_Wait_ac(activeCollection, timeOut) {}
  virtual Goal::Goal_Status Process(double dTime);
  void Terminate();
};
/* Goal_DriveWithTimer
    * This goal implements Goal_Wait to drive at specified speed until time runs out.
    */
class Goal_DriveWithTimer : public Goal_Wait_ac
{
public:
  Goal_DriveWithTimer(ActiveCollection *activeCollection, double leftSpeed, double rightSpeed, double timeOut) : Goal_Wait_ac(activeCollection, timeOut)
  {
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
  }
  Goal::Goal_Status Process(double dTime);
  void Terminate();

private:
  double m_leftSpeed, m_rightSpeed;
};
#pragma endregion TimerGoals
/* Goal_ControllerOverride
 *  Test if driver or operator or both are pressing controls, if so, fail.
 *  0 = driver, 1 = operator, 2 = both
 * 
 * This Goal will mostly be added into multitask goals to check if failed while other goals are also running.
 */
class Goal_ControllerOverride : public AtomicGoal
{
  public:
    Event DriverValueChanged;
    Event OperatorValueChanged;
    bool m_IsDriveInUse = false;
    bool m_IsOperatorInUse = false;
    Goal_ControllerOverride(ActiveCollection *activeCollection, int controller = 2)
    {
      m_activeCollection = activeCollection;
      m_controller = controller;
      SetCallbacks(true);
    }
    virtual void Activate();
    virtual Goal::Goal_Status Process(double dTime);
    virtual void Terminate();
    virtual void SetCallbacks(bool bind);
  private:
    virtual void TestDriver();
    virtual void TestOperator();
    int m_controller;
    ActiveCollection *m_activeCollection;
};



//Goals that use data to determine completion go here
#pragma region FeedbackLoopGoals
/* Goal_Turn
    * This goal uses the navx to turn to a specified degree
    * 
    */


class AutoPath : public CompositeGoal
{
  public:
    AutoPath(ActiveCollection* activeCollection, Auto Path)
    {
      lenght = Path.Num;
      m_activeCollection = activeCollection;
      Dist = new double[Path.Num];
      Angle = new double[Path.Num];
      Actions = new double[Path.Num];
      SpeedB = new double[Path.Num];
      Dist[0] = 0;
      Angle[0] = 0;
      Actions[0] = 0;
      for (int i = 1; i < Path.Num; i++)
      {
        Actions[i] = Path.Waypoints[i].Act;
        SpeedB[i] = Path.Waypoints[i].Speed;
        float Dis = sqrt(pow(Path.Waypoints[i].X - Path.Waypoints[i-1].X, 2) + pow(Path.Waypoints[i].Y - Path.Waypoints[i-1].Y, 2));
            
        float XDIS = (Path.Waypoints[i].X - Path.Waypoints[i-1].X);
        float YDIS = (Path.Waypoints[i].Y - Path.Waypoints[i-1].Y);

        float A = (atan2(YDIS, XDIS) / 3.1415) * 180;

        Dist[i] = Dis;
        Angle[i] = A;
      }
      for (int i = Path.Num - 1; i > 1; i--)
      {
        Angle[i] -= Angle[i - 1];

        if (ABSValue(Angle[i]) == 180)
        {
          Angle[i] = 0;
          Dist[i] *= -1;
          if (i != Path.Num - 1)
          {
            Angle[i + 1] += 180;
          }
        }
        if (Angle[i] < -200)
          Angle[i] = GetMax(Angle[i], Angle[i] + 360);
        else if (Angle[i] > 200)
          Angle[i] = GetMin(Angle[i], Angle[i] - 360);
      }
      for(int i = 0; i < 10; i++)
        {
          Log::General(to_string(Angle[i]) + "Degrees");      
          Log::General(to_string(Dist[i]) + "Feet");
        }
    }
    virtual void Activate();
  private:
    double* Dist;
    double* Angle;
    double* Actions;
    double* SpeedB;
    int lenght = 0;
    ActiveCollection* m_activeCollection;
};

class Goal_MoveForward : public AtomicGoal
{
  public:
    Goal_MoveForward(ActiveCollection *activeCollection, double Dist, double MaxPowerOutput, double MaxTime, double SpeedBias = 1)
    {
        RealTarget = Dist * 5;
        MaxPower = MaxPowerOutput;
        m_activeCollection = activeCollection;
        distTo = (RealTarget);
        TotalTime = MaxTime;
        IsNegative = Dist < 0;
        //enc0 = activeCollection->GetEncoder("enc0");
        enc0 = (SparkMaxItem*)activeCollection->Get("left1");
        m_Status = eInactive;
        SBias = SpeedBias;
    }

    virtual void Activate();
    virtual Goal::Goal_Status Process(double dTime);
    virtual void Terminate();

    private:
      double SBias;
      double TimePassed = 0;
      double TotalTime = 0;
      double RealTarget = 0;         // 85 was working yesterday
	    double MaxPower = 0;

      ActiveCollection *m_activeCollection;
      //EncoderItem* enc0;
      SparkMaxItem *enc0;
      NavX *navx;

      bool IsNegative  = false;
      double P = 5; //PID constants
	    double I = 0.5;
	    double D = 0;
	    double PE = 2; //PID constants
	    double IE = 0.8;
	    double DE = 0.05;
	    double Limit = 0.5;
      double Pevpower = 0;
      double PrevEResult = 0;
	    double MinPower = 0;
	    double PrevE = 0, totalE = 0;
	    double PrevEncoder = 0, totalEncoder = 0, ErrorTo = 0;
	    double enc = 0;
      double currentValue = 0;
	    double distTo = 0;
      double BiasE = 0;
      double Bias = 0;
	    double NumberAtTarget = 0;
      bool Moving = false;
      bool Done = false;
};

class Goal_TurnPIDF : public AtomicGoal
{
  public:
    Goal_TurnPIDF(ActiveCollection *activeCollection, double Angle, double MaxPowerOutput, double MaxTime, double SpeedBias = 1)
    {
        navx = activeCollection->GetNavX();
        RealTarget = ABSValue(Angle);
        MaxPower = MaxPowerOutput;
        m_activeCollection = activeCollection;
        TotalTime = MaxTime;
        m_Status = eInactive;
        SBias = SpeedBias;
    }

    virtual void Activate();
    virtual Goal::Goal_Status Process(double dTime);
    virtual void Terminate();

    private:
      double SBias;
      double TimePassed = 0;
      double TotalTime = 0;
      double RealTarget = 0;
	    double MaxPower = 0;
      double Pevpower = 0;

      ActiveCollection *m_activeCollection;
      NavX *navx;

      float Offset = 0;
      bool IsNegative  = false;
      double P = 15; //PID constants
	    double I = 15;
	    double D = 1;
      double Bias = 0;
	    double Limit = 0.35;
	    double MinPower = 0;
	    double PrevE = 0, totalE = 0, ErrorTo  = 0;
      double currentValue = 0;

	    double NumberAtTarget = 0;
      bool Moving = false;
      bool Done = false;
};


#pragma endregion
#pragma endregion

//Composite Goals go here
#pragma region CompositeGoals

//utility-style goals go here
#pragma region utility

#pragma endregion

//auton goals and other complex goals go here
#pragma region complex



/* Goal_WaitThenDrive
    * This is just a test composite goal. Unlikely it will be used IRL
    */
class Goal_WaitThenDrive : public CompositeGoal
{
public:
  Goal_WaitThenDrive(ActiveCollection *activeCollection, double leftSpeed, double rightSpeed, double waitTime, double driveTime)
  {
    m_activeCollection = activeCollection;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    m_waitTime = waitTime;
    m_driveTime = driveTime;
    m_Status = eInactive;
  }
  virtual void Activate();

private:
  ActiveCollection *m_activeCollection;
  double m_leftSpeed, m_rightSpeed, m_waitTime, m_driveTime;
};
#pragma endregion

#pragma endregion

//multitask goals go here
#pragma region MultitaskGoals

#pragma endregion

class Position : public AtomicGoal
{
public:
  Position(ActiveCollection *activeCollection, double Target, double MaxTime)
  {
    Spinner = (TalonSRXItem*)activeCollection->Get("Spinner");
    m_Status = eInactive;
    m_Calculate = Target * 100;
    MaxT = MaxTime;
    m_Status = eInactive;
  }
  virtual Goal::Goal_Status Process(double dTIme);
  virtual void Terminate();
  virtual void Activate();

private:
  TalonSRXItem* Spinner;
  double m_Calculate;
  double Bias = 0;
  double P = 10;
  double I = 0.01;
  double D = 0.0001;
  double LastResult = 0.01;
  double TotalE = 0;
  double LastE = 0;
  double CurrentT = 0;
  double MaxT;
};

class Goal_Intake : public AtomicGoal
{
  public:
    Goal_Intake(ActiveCollection *activeCollection, double In_Speed, bool Deploy)
    {
      IntakeMotor = (VictorSPItem*)activeCollection->Get("Intake");
      Floor = (VictorSPXItem*)activeCollection->Get("Floor");
      IndexL = (VictorSPXItem*)activeCollection->Get("indexer0");
      IndexR = (VictorSPXItem*)activeCollection->Get("indexer1");
      Wrist = (DoubleSolenoidItem*)activeCollection->Get("IntakeDeploy");
      Sped = In_Speed;
      DeployIntake = Deploy;
      m_Status = eInactive;
     }
     virtual Goal::Goal_Status Process(double dTime);
     virtual void Terminate();
     virtual void Activate();
  private:
    bool DeployIntake = false;
    double Sped;
    VictorSPItem* IntakeMotor;
    VictorSPXItem* Floor;
    VictorSPXItem* IndexL;
    VictorSPXItem* IndexR;
    DoubleSolenoidItem* Wrist;
};

class Goal_REVColorSensorV3 : public AtomicGoal
{
public:
  Goal_REVColorSensorV3(ActiveCollection *activeCollection, string ColorVariable, double MaxTime)
  {
    Spinner = (TalonSRXItem*)activeCollection->Get("Spinner");
    TargetString = (ColorVariable == "Blue" ? 1 : (ColorVariable == "Red" ? 2 : (ColorVariable == "Green" ? 3 : (ColorVariable == "Yellow" ? 4 : 0))));
    Color = (REVColorSensorV3*)activeCollection->Get("Color");
    m_Status = eInactive;
    MaxT = MaxTime;
  }

  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();
  virtual void Activate();

  private:
    TalonSRXItem* Spinner;
    int TargetString = 0;
    REVColorSensorV3* Color;
    double CurrentT;
    double MaxT;
};

class Goal_ShooterYeet : public AtomicGoal
{
public:
  Goal_ShooterYeet(ActiveCollection *activeCollection, double MaxSpeed, string MotorName1, string MotorName2)
  {
    m_activeCollection = activeCollection;
    m_MaxSpeed = MaxSpeed;
    double SpeedTar = 1;
    ShooterMotor = (TalonSRXItem*)activeCollection->Get(MotorName1);
    ShooterMotor2 = (TalonSRXItem*)activeCollection->Get(MotorName2);
    IsNegative = SpeedTar < 0;
    m_Status = eInactive;
  }
  //Find what motor to get

  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();
  double m_Speed = 0;
  TalonSRXItem *ShooterMotor;
  TalonSRXItem *ShooterMotor2;
  double revSpeed = 0;
  double SpedSpeed = 0;
  bool Reached = false;
  bool Shoot_DA_BOOL = false;
  bool LimeLightTrack = false;
  
private:
  double m_MaxSpeed;
  
  double Bias = 10;
  
  double LastE = 0;
  double P = 2;
  double I = 50;
  double D = 0;
  double LastResult = 0;
  
  double total = 0;
  double PrevE = 0;
  bool IsNegative;
  double lastPos = 0;
  double LastSpe = 0;
  double SlowDownBias = 0.1;
  bool FirstRun = true;

  ActiveCollection* m_activeCollection;
};

class Goal_ShooterBunch : public AtomicGoal
{
public:
  Goal_ShooterBunch(ActiveCollection *activeCollection, double ShootSpeed)
  {
    MovingFloor = (VictorSPXItem*)activeCollection->Get("Floor");
    IndexL = (VictorSPXItem*)activeCollection->Get("indexer0");
    IndexR = (VictorSPXItem*)activeCollection->Get("indexer1");
    Valve = (DoubleSolenoidItem*)activeCollection->Get("Valve");
    Lime = (limelight*)activeCollection->Get("LimeLight");
    ShootWheel = new Goal_ShooterYeet(activeCollection, 1, "Shooter0", "Shooter1");
    (ShootSpeed > 0 ? ShootWheel->m_Speed = ShootSpeed : 0);
    (ShootSpeed > 0 ? ShootWheel->LimeLightTrack = false : ShootWheel->LimeLightTrack = true);
    m_Status = eInactive;
  }

  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();
  int numShots = 0;

private:
  double CurrentTime;
  double Speed = 0.6;
  bool Increment = false;
  bool Prep = false;
  bool Shoot = false;
  DoubleSolenoidItem* Valve;
  VictorSPXItem* MovingFloor;
  VictorSPXItem* IndexL;
  VictorSPXItem* IndexR;
  Goal_ShooterYeet* ShootWheel;
  limelight* Lime;
};