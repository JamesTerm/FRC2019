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
#include "Goal.h"
#include "../Util/VisionTarget.h"
#include "../Util/Units/Distances.h"

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
#pragma endregion
#if 0
#pragma region ControllerOverride
/* Goal_ControllerOverride
 *  Test if driver or operator or both are pressing controls, if so, fail.
 *  0 = driver, 1 = operator, 2 = both
 * 
 * This Goal will mostly be added into multitask goals to check if failed while other goals are also running.
 */
class Goal_ControllerOverride : public AtomicGoal
{
  public:
    void SetCallbacks(bool bind);
    Goal_ControllerOverride(Framework::Base::EventMap &em) : m_EventMap(em)
    {	m_controller = 2;
		SetCallbacks(true);
    }
    //0 = driver, 1 = operator, 2 = both
    Goal_ControllerOverride(Framework::Base::EventMap &em,int controller) : m_EventMap(em)
    {	m_controller = controller;
		SetCallbacks(true);
    }
	~Goal_ControllerOverride()
	{	SetCallbacks(false);
	}
    virtual void Activate();
    virtual Goal::Goal_Status Process(double dTime);
    //virtual void Terminate();
  private:
	void TestDriver();
	void TestOperator();
	Framework::Base::EventMap &m_EventMap;
    int m_controller;
	bool m_IsOperatorInUse = false;
	bool m_IsDriveInUse = false;
};
#pragma endregion
#endif
class Goal_ControllerOverride : public AtomicGoal
{
  public:
    Event DriverValueChanged;
    Event OperatorValueChanged;
    bool m_IsDriveInUse;
    bool m_IsOperatorInUse;
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
class Goal_ElevatorControl : public AtomicGoal
{
  public:
    Goal_ElevatorControl(ActiveCollection* activeCollection, double target)
    {
      m_activeCollection = activeCollection;
      m_target = target;
      m_Status = eInactive;
      m_potientiometer = (PotentiometerItem*)activeCollection->Get("pot"); //TODO this
    }
    virtual void Activate();
    virtual Goal::Goal_Status Process(double dTime);
    virtual void Terminate();
  private:
    double m_target;
    double m_currentPos;
    double m_power;
    ActiveCollection* m_activeCollection;
    PotentiometerItem* m_potientiometer;

    int kp = 0, ki = 0, kd = 0;
    int bias = 0; //bias is a constant that is added to the output.
    double error, integ, deriv;
    double errorPrior;

};
    

//Goals that use data to determine completion go here
#pragma region FeedbackLoopGoals
/* Goal_Turn
    * This goal uses the navx to turn to a specified degree
    * 
    */
class Goal_Turn : public Goal_Wait_ac
{
public:
  Goal_Turn(ActiveCollection *activeCollection, double angle, double timeOut = 3.0) : Goal_Wait_ac(activeCollection, timeOut) 
  {
    m_navx = activeCollection->GetNavX();
  }
  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();

private:
  double m_target;
  double m_power;
  const double m_freedom = 1; //code can be within +/- n degrees of target
  NavX *m_navx;

  double kp = 1, ki = 0, kd = .05;
  double bias = 0; //bias is a constant that is added to the output. We most likely will not use it but it is here if needed.
  double propError, integ, deriv;
  double errorPrior;
};

/* Goal_DriveStraight
    * This goal uses encoder to drive a specified amount forward and navx to stay straight
    */
class Goal_DriveStraight : public Goal_Wait_ac
{
public:
  //*standard constructor
  Goal_DriveStraight(ActiveCollection *activeColelction, double dist, double power, double timeOut) : Goal_Wait_ac(activeColelction, timeOut)
  {
    m_distTarget = dist;
    m_encLeft = m_activeCollection->GetEncoder("enc0");
    m_encRight = m_activeCollection->GetEncoder("enc1");
    m_leftPower = m_rightPower = power;
  }
  //*optional constructor without timeOut, calls standard with default timeOut value
  Goal_DriveStraight(ActiveCollection *activeCollection, double dist, double power) : Goal_DriveStraight(activeCollection, dist, power, 7.0) {}
  //*constructor that takes a distance object as a parameter
  Goal_DriveStraight(ActiveCollection *activeCollection, Distance *dist, double power, double timeOut) : Goal_DriveStraight(activeCollection, dist->inATick(), power, timeOut) {}
  //*constructor that takes distance object and uses default timeOut value
  Goal_DriveStraight(ActiveCollection *activeCollection, Distance *dist, double power) : Goal_DriveStraight(activeCollection, dist, power, 7.0) {}

  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();

private:
  double m_distTarget;
  double m_distTraveled = 0;
  double m_leftPower, m_rightPower;
  EncoderItem *m_encLeft, *m_encRight;

  double kp = 0, ki = 0, kd = 0;
  double bias = 0; //bias is a constant that is added to the output. We most likely will not use it but it is here if needed.
  double error, integ, deriv;
  double errorPrior;
};

//TODO vision aligment:
/* if x is out of bounds, turn towards target
 * then, if radius (distance) out of bounds, drive straight towards target 
 * SetDrive() not turn goal
 */
class Goal_VisionAlign : public Goal_TimeOut
{
public:
  Goal_VisionAlign(ActiveCollection *activeCollection, VisionTarget *target, double timeOut = 7.0) : Goal_TimeOut(activeCollection, timeOut)
  {
    m_inst = nt::NetworkTableInstance::GetDefault(); //!Network tables
    m_visionTable = m_inst.GetTable("VISION_2019");  //!Vision network table
    m_target = target;
    m_currentTarget = new VisionTarget();
  }

  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();

private:
  void updateVision()
  {
    m_currentTarget->setX(m_visionTable->GetNumber("X", 0));
    m_currentTarget->setY(m_visionTable->GetNumber("Y", 0));
    m_currentTarget->setRadius(m_visionTable->GetNumber("RADIUS", 0));
    //Area = m_visionTable->GetNumber("AREA", 0);
    Height = m_visionTable->GetNumber("HEIGHT", 0);
    Width = m_visionTable->GetNumber("WIDTH", 0);
    HasTarget = m_visionTable->GetBoolean("HASTARGET", false);
  }
  int Height, Width;
  bool HasTarget;

  //constants for motor speeds. These are multiplied by vision error
  const double TURN_KP = 0.0025;   //?guess for right now
  const double STRAIGHT_KP = 0.05; //?guess for right now

  nt::NetworkTableInstance m_inst;        //!Network tables
  shared_ptr<NetworkTable> m_visionTable; //!Vision table
  VisionTarget *m_target;
  VisionTarget *m_currentTarget;
};

// /* Goal_ElevatorPosition
//  * Positions: 0, 1, 2, 3
//  * 0 = all the way down, 3 = all the way up
//  * Offset: small increase in height for cargo intake on rocket
//  */
// class Goal_ElevatorPosition
// {
//   public:

// };
#pragma endregion

#pragma region UtilGoals

/* Goal_Hatch
    This goal is meant to manipulate the mechanisms that would result in outtaking the hatch (this will need adjusting)
    */
class Goal_Hatch : public Goal_Wait_ac
{
public:
  Goal_Hatch(ActiveCollection *activeCollection, double timeOut) : Goal_Wait_ac(activeCollection, 1)
  {
    m_Status = eActive;
    m_timeOut = timeOut;
    m_currentTime = 0; ///////////////////////////////////////////////////////////////////////////////////////////////////
  }
  virtual void Activate();
  virtual Goal::Goal_Status Process(double dTime);
  virtual void Terminate();

protected:
  ActiveCollection *m_activeCollection;
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

/* Goal_OneHatch
    This goal is meant to score one hatch on the cargo during autonomous
    */
class Goal_OneHatchFrontShip : public CompositeGoal
{
public:
  Goal_OneHatchFrontShip(ActiveCollection *activeCollection, string position = "none")
  {
    m_activeCollection = activeCollection;
    m_position = position;
    m_Status = eInactive;
  }

  virtual void Activate();

private:
  string m_position;
  ActiveCollection* m_activeCollection;
};

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