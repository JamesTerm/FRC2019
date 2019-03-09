/****************************** Header ******************************\
Class Name: (multiple classes)
File Name: FRC2019_Goals.cpp
Summary: All implemented Goals go here. Region statements categorize
types
Project: BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Chris Weeks
Email: chrisrweeks@aol.com
\********************************************************************/
#include "FRC2019_Goals.h"

using namespace std;

//?HINT ctrl+k then ctrl+0 will collapse all regions
//?ctrl+k then ctrl+j will uncollapse all regions





#pragma region AtomicGoals
#pragma region TimerGoals
/***********************Goal_Wait_ac***********************/
void Goal_Wait_ac::Activate()
{
    m_Status = eActive;
}
Goal::Goal_Status Goal_Wait_ac::Process(double dTime)
{
    if (m_Status == eActive)
    {
        m_currentTime += dTime;
        if (m_currentTime >= m_timeOut)
        {
            Terminate();
            return m_Status = eCompleted;
        }

        return eActive;
    }
    else
    {
        return m_Status;
    }
}

void Goal_Wait_ac::Terminate()
{
}
/***********************Goal_TimeOut***********************/

Goal::Goal_Status Goal_TimeOut::Process(double dTime)
{
    if (m_Status == eActive)
    {
        m_currentTime += dTime;
        if (m_currentTime >= m_timeOut)
        {
            Terminate();
            return m_Status = eFailed;
        }
        //cout << m_activeCollection->GetNavX()->GetAngle() << endl; //DEBUG
        return eActive;
    }
    else
    {
        return m_Status;
    }
}
void Goal_TimeOut::Terminate()
{
}
/***********************Goal_DriveWithTimer***********************/
Goal::Goal_Status Goal_DriveWithTimer::Process(double dTime)
{
    if (m_Status == eActive)
    {
        m_currentTime += dTime;
        SetDrive(m_leftSpeed, m_rightSpeed, m_activeCollection);
        if (m_currentTime >= m_timeOut)
        {

            Terminate();
            return m_Status = eCompleted;
        }
        return m_Status = eActive;
    }
    else
    {
        return m_Status;
    }
}

void Goal_DriveWithTimer::Terminate()
{
    StopDrive(m_activeCollection);
}
#pragma endregion

#pragma region ControllerOverride
#if 0
/***********************Goal_ControllerOverride***********************/
void Goal_ControllerOverride::Activate()
{
    m_Status = eActive;
}

void Goal_ControllerOverride::TestDriver()
{
	if (m_IsDriveInUse)
	{
		//since we listened to just one event this is a bit redundant added for consistency
		m_EventMap.Event_Map["DriverDetected"].Fire();  
		m_Status = eCompleted;
	}
}

void Goal_ControllerOverride::TestOperator()
{
	if (m_IsOperatorInUse)
	{
		m_EventMap.Event_Map["OperatorDetected"].Fire();
		m_Status = eCompleted;
	}
}

//Setup to remain active until an override is detected... in which case it can fire an event and change to completed status
Goal::Goal_Status Goal_ControllerOverride::Process(double dTime)
{
    if(eActive)
    {
		if (m_controller == 0)
			TestDriver();
		else if (m_controller == 1)
			TestOperator();
        else
        {
			TestDriver();
			if (m_Status==eActive)
				TestOperator();
        }
    }
	return m_Status;
}


void Goal_ControllerOverride::SetCallbacks(bool bind)
{
	std::function<void()> oe = [&] 
	{
		m_IsOperatorInUse=true; 
	};
	std::function<void(bool)> oe_on_off = [&](bool on)
	{
		m_IsOperatorInUse = true;
	};
	std::function<void(double)> oe_val = [&](double val)
	{
		if (val>0.0)
			m_IsOperatorInUse = true;
	};

	std::function<void(bool)> de_on_off = [&](bool on)
	{
		m_IsDriveInUse = on;
	};

	if (bind)
	{
		m_EventMap.EventOnOff_Map["IsDriverMoving"].Subscribe(this, de_on_off);
		m_EventMap.Event_Map["Arm_SetPosRest"].Subscribe(this,oe);
		m_EventMap.Event_Map["Arm_SetPosCargo1"].Subscribe(this, oe);
		m_EventMap.Event_Map["Arm_SetPosCargo2"].Subscribe(this, oe);
		m_EventMap.Event_Map["Arm_SetPosCargo3"].Subscribe(this, oe);
		m_EventMap.EventValue_Map["Arm_SetCurrentVelocity"].Subscribe(this, oe_val);
		m_EventMap.EventOnOff_Map["Arm_IntakeDeploy"].Subscribe(this, oe_on_off);
		//note skipping keyboard, but could indeed listen to them as well
		m_EventMap.EventValue_Map["Claw_SetCurrentVelocity"].Subscribe(this, oe_val);
		m_EventMap.EventOnOff_Map["Arm_HatchDeploy"].Subscribe(this, oe_on_off);
		m_EventMap.EventOnOff_Map["Arm_HatchGrabDeploy"].Subscribe(this, oe_on_off);
		m_EventMap.EventOnOff_Map["Robot_CloseDoor"].Subscribe(this, oe_on_off);
	}
	else
	{
		m_EventMap.EventOnOff_Map["IsDriverMoving"].Remove(this);
		m_EventMap.Event_Map["Arm_SetPosRest"].Remove(this);
		m_EventMap.Event_Map["Arm_SetPosCargo1"].Remove(this);
		m_EventMap.Event_Map["Arm_SetPosCargo2"].Remove(this);
		m_EventMap.Event_Map["Arm_SetPosCargo3"].Remove(this);
		m_EventMap.EventValue_Map["Arm_SetCurrentVelocity"].Remove(this);
		m_EventMap.EventOnOff_Map["Arm_IntakeDeploy"].Remove(this);
		m_EventMap.EventValue_Map["Claw_SetCurrentVelocity"].Remove(this);
		m_EventMap.EventOnOff_Map["Arm_HatchDeploy"].Remove(this);
		m_EventMap.EventOnOff_Map["Arm_HatchGrabDeploy"].Remove(this);
		m_EventMap.EventOnOff_Map["Robot_CloseDoor"].Remove(this);
	}
}
#endif
void Goal_ControllerOverride::Activate()
{
    m_Status = eActive;
}

Goal::Goal_Status Goal_ControllerOverride::Process(double dTime)
{
    if(eActive)
    {
		if (m_controller == 0)
			TestDriver();
		else if (m_controller == 1)
			TestOperator();
        else
        {
			TestDriver();
			if (m_Status==eActive)
				TestOperator();
        }
    }
	return m_Status;
}

void Goal_ControllerOverride::Terminate()
{
    SetCallbacks(false);
}

void Goal_ControllerOverride::TestDriver()
{
    if (m_IsDriveInUse)
	{
		DriverValueChanged(new SenderEventArgs<Goal_ControllerOverride*>(this));  
		m_Status = eFailed;
	}
}

void Goal_ControllerOverride::TestOperator()
{
	if (m_IsOperatorInUse)
	{
		OperatorValueChanged(new SenderEventArgs<Goal_ControllerOverride*>(this));
		m_Status = eFailed;
	}
}

void Goal_ControllerOverride::SetCallbacks(bool bind)
{
    auto onValueChanged = [&](EventArgs* e) {
        try{
            auto args = (TEventArgs<double, Controls::ControlItem*>*)e;
            if(args->GetSender()->joy->GetPort() == 0){
                m_IsDriveInUse = true;
            }
            if(args->GetSender()->joy->GetPort() == 1){
                m_IsOperatorInUse = true;
            }
            delete args;
            args = nullptr;
        }catch(exception &e){
            Log::Error("Known Exception Thrown in onValueChanged in a ControllerOverride! This can cause fatal Runtime Errors! Check your logs and XML.");
            Log::Error(e.what());
        }catch(...){
            Log::Error("UnknownException Thrown in onValueChanged in ControllerOverride! This can cause fatal Runtime Errors! Check your XML and yell at the programmers!");
	    }
    };

    if(bind){
        for(Event *e : m_activeCollection->EventMap)
            (*e).subscribeFirstPriority(onValueChanged);
    }else{
        for(Event *e : m_activeCollection->EventMap)
            (*e).unsubscribeFirstPriority();
    }
}
#pragma endregion

#pragma region FeedbackLoopGoals
/***********************Goal_Turn***********************/
void Goal_Turn::Activate()
{
    Goal_Wait_ac::Activate();
    m_navx->Reset();
}

Goal::Goal_Status Goal_Turn::Process(double dTime)
{
    if (m_Status == eActive)
    {
        m_currentTime += dTime;
        if (m_currentTime > m_timeOut)
        {
            Terminate();
            cout << "no target" << endl;
            //return m_Status = eFailed; //set m_Status to failed and return m_Status in one line
        }

        double lowerBound = m_target - m_freedom;
        double upperBound = m_target + m_freedom;
        double currentAngle = m_navx->GetAngle();
        //cout << "ANGLE: " << currentAngle << endl;

        propError = (m_target - currentAngle) / m_target;
        integ += propError * dTime;               //Right Riemann Sum integral
        deriv = (propError - errorPrior) / dTime; // rise/run slope
        errorPrior = propError;                   //set errorPrior for next process call

        m_power = bias + (kp * propError) + (ki * integ) + (kd * deriv); //power is equal to P,I,D * k-values + bias

        if (currentAngle < lowerBound || currentAngle > upperBound)
        {
            SetDrive(m_power, -m_power, m_activeCollection);
            return m_Status = eActive;
        }
        else
        {
            Terminate();
            return m_Status = eCompleted;
        }
    }
    else
    {
        return m_Status;
    }
}

void Goal_Turn::Terminate()
{
    StopDrive(m_activeCollection);
}

/***********************Goal_DriveStraight***********************/
void Goal_DriveStraight::Activate()
{
    Goal_Wait_ac::Activate();
	if (m_encLeft)
	{
		m_encLeft->Reset();
		m_encRight->Reset();
	}
	else
		m_Status = Goal::eFailed;
}

//Left enc = master, right enc = slave
Goal::Goal_Status Goal_DriveStraight::Process(double dTime)
{
    if (m_Status = eActive)
    {
		SmartDashboard::PutBoolean("DRIVE STRAIGHT STATUS", true);
        m_currentTime += dTime;
        if (m_currentTime > m_timeOut)
            return m_Status = eFailed;
        m_distTraveled += m_encLeft->Get();
        if (m_distTraveled >= m_distTarget)
        {
            Terminate();
            return m_Status = eCompleted;
        }

        //TODO PID

        error = m_encLeft->Get() - m_encRight->Get(); //we cannot use true propError here because target is zero.
        integ += error * dTime;                       //Right Riemann Sum integral
        deriv = (error - errorPrior) / dTime;         // rise/run slope
        errorPrior = error;                           //set errorPrior for next process call

        //left power never changes
        m_rightPower += bias + (error * kp) + (integ * ki) + (deriv * kd);
        SetDrive(m_leftPower, m_rightPower, m_activeCollection);

        m_encLeft->Reset();
        m_encRight->Reset();
    }
    else
    {
		SmartDashboard::PutBoolean("DRIVE STRAIGHT STATUS", true);
        return m_Status;
    }
}

void Goal_DriveStraight::Terminate()
{
    StopDrive(m_activeCollection);
}
/***********************Goal_VisionAlign***********************/
void Goal_VisionAlign::Activate()
{
    cout << "goal active" << endl;
    m_Status = eActive;
    updateVision();
}

Goal::Goal_Status Goal_VisionAlign::Process(double dTime)
{
    m_currentTime += dTime;
    if(m_currentTime > m_timeOut)
    {
        cout << "time out" << endl;
        Terminate();
        return m_Status = eFailed;
    }
    updateVision();
    cout << m_currentTarget->getX() << " " << m_currentTarget->getY() << " " << m_currentTarget->getRadius() << " " << Height << " " << Width << HasTarget << endl;
    if(!HasTarget) 
    {
        cout << "no target" << endl;
        StopDrive(m_activeCollection);
        return m_Status = eActive; //!return failed for real thing or search
    }
    if(m_target->compareX(m_currentTarget) < -20) 
    {
        cout << "turn left" << endl;
        SetDrive(m_target->compareX(m_currentTarget) * TURN_KP,-(m_target->compareX(m_currentTarget) * TURN_KP),m_activeCollection);
    }
    else if(m_target->compareX(m_currentTarget) > 20)
    {
        cout << "turn right" << endl;
        SetDrive(m_target->compareX(m_currentTarget) * TURN_KP,-(m_target->compareX(m_currentTarget) * TURN_KP),m_activeCollection);
    }
    else
    {
        if(m_target->compareRadius(m_currentTarget) < -2 || m_target->compareRadius(m_currentTarget) > 2)
        {
            cout << "drive" << endl;
            SetDrive(m_target->compareRadius(m_currentTarget) * STRAIGHT_KP,(m_target->compareRadius(m_currentTarget) * STRAIGHT_KP),m_activeCollection);
        }
        else
        {
            cout << "aligned" << endl;
            Terminate();
            //return m_Status = eCompleted;
        }
        
    }
    //cout << "nothing" << endl;
    return m_Status = eActive;

}

void Goal_VisionAlign::Terminate()
{
    StopDrive(m_activeCollection);
}

#pragma endregion

#pragma region UtilGoals
/***********************Goal_Hatch***********************/
void Goal_Hatch::Activate()
{
    m_Status = eActive;
}

Goal::Goal_Status Goal_Hatch::Process(double dTime)
{
    
}
void Goal_Hatch::Terminate()
{
}

#pragma endregion
#pragma endregion

#pragma region CompositeGoals
/***********************Goal_WaitThenDrive***********************/
void Goal_WaitThenDrive::Activate()
{
    AddSubgoal(new Goal_Wait_ac(m_activeCollection, m_waitTime));
    AddSubgoal(new Goal_DriveWithTimer(m_activeCollection, m_leftSpeed, m_rightSpeed, m_driveTime));

    m_Status = eActive;
}

/***********************Goal_OneHatch***********************/
void Goal_OneHatchFrontShip::Activate()
{
    m_Status = eActive;
    if(m_position == "Level 1 Left")
    {
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(5.0), .75));
        AddSubgoal(new Goal_Turn(m_activeCollection, 90));
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(3.0), .75));
        AddSubgoal(new Goal_Turn(m_activeCollection, -90));
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(2.0), .5));
        //GOAL DEPLOY HATCH
    }
    else if(m_position == "Level 1 Center")
    {
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(7.0), .75));
        //deploy hatch
    }
    else if(m_position == "Level 1 Right")
    {
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(5.0), .75));
        AddSubgoal(new Goal_Turn(m_activeCollection, -90));
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(3.0), .75));
        AddSubgoal(new Goal_Turn(m_activeCollection, 90));
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(2.0), .5));
        //GOAL DEPLOY HATCH
    }
    else if(m_position == "Level 2 Left")
    {
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(5.0), .75));
        AddSubgoal(new Goal_OneHatchFrontShip(m_activeCollection, "Level 1 Left"));
    }
    else if(m_position == "Level 2 Right")
    {
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(5.0), .75));
        AddSubgoal(new Goal_OneHatchFrontShip(m_activeCollection, "Level 1 Right"));
    }
    else
    {
        AddSubgoal(new Goal_DriveStraight(m_activeCollection, new Feet(10.0), .75));
    }
    
}
#pragma endregion

#pragma region MultitaskGoals

#pragma endregion