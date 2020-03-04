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
using namespace Util;

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
        Log::General("timeOutProcess");
        m_currentTime += dTime;
        SmartDashboard::PutNumber("timeOut",m_currentTime);
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
    Log::General("b4");
    if (m_Status == eActive)
    {
        Log::General("processing");
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

void Goal_ElevatorControl::Activate()
{
    m_Status = eActive;
    error = 0;
    deriv = 0;
    integ = 0;
}

#if 0
Goal::Goal_Status Goal_ElevatorControl::Process(double dTime)
{
    if(m_Status == eActive)
    {
        m_timeElapsed += dTime;
        if(((m_pot->Get() > m_target && m_goingUp) || (m_pot->Get() < m_target && !m_goingUp)))
        {
            Terminate();
            return m_Status = eCompleted;
        }
        else
        {
            //look at elevator trapezoid in design folder
            if(m_goingUp)
            {
                if(m_timeElapsed < .25)
                {
                    SetElevator((MAX_POWER-MIN_POWER)/.25 * m_timeElapsed + MIN_POWER, m_activeCollection);
                }
                else if(m_timeElapsed > .25 && m_timeElapsed < moveTime)
                {
                    SetElevator(MAX_POWER, m_activeCollection);
                }
                else if(m_timeElapsed > moveTime && m_timeElapsed < moveTime + .75)
                {
                    SetElevator(((MIN_POWER-MAX_POWER)/.25)*(m_timeElapsed - moveTime - .25) + MAX_POWER, m_activeCollection);
                }
                else
                {
                    Terminate();
                    return m_Status = eCompleted;
                }
                
                
            }
            else
            {
                //exact same math as above but with a negative in front
                if(m_timeElapsed < .25)
                {
                    SetElevator(-((MAX_POWER-MIN_POWER)/.25 * m_timeElapsed + MIN_POWER), m_activeCollection);
                }
                else if(m_timeElapsed > .25 && m_timeElapsed < moveTime)
                {
                    SetElevator(-MAX_POWER, m_activeCollection);
                }
                else if(m_timeElapsed > moveTime && m_timeElapsed < moveTime + .25)
                {
                    SetElevator(-(((MIN_POWER-MAX_POWER)/.25)*(m_timeElapsed - moveTime - .25) + MAX_POWER), m_activeCollection);
                }
                else
                {
                    Terminate();
                    return m_Status = eCompleted;
                }
            }
        }
    }
    else
    {
        return m_Status;
    }
    
}
#else 
Goal::Goal_Status Goal_ElevatorControl::Process(double dTime)
{
    if(m_Status == eActive)
    {
        Log::General("ele target: " + to_string(m_target));
#ifndef _Win32
        m_currentPos = m_pot->Get();
        #else 
        m_currentPos = .6;
        #endif
        if(m_currentPos >= m_target - FREEDOM && m_currentPos <= m_target + FREEDOM)
        {
            Terminate();
            nt::NetworkTableInstance::GetDefault().GetTable("DASHBOARD_TABLE")->PutNumber("pot",m_currentPos);
            return eCompleted;
        }
        error = m_target - m_currentPos;
        integ += error * dTime;
        deriv = (error - errorPrior) / dTime;

        double power = kp * error + ki * integ + kd * deriv;
        if(power > MAX_POWER) power = MAX_POWER;
        if(power < -MAX_POWER) power = -MAX_POWER;
        SetElevator(power, m_activeCollection);
        return eActive;

    }
    else
    {
        return m_Status;
    }
    
}
#endif

void Goal_ElevatorControl::Terminate()
{
    StopElevator(m_activeCollection);
}

void Goal_RelativeElevatorControl::Activate()
{
    goal->Activate();
    m_Status = eActive;
}

Goal::Goal_Status Goal_RelativeElevatorControl::Process(double dTime)
{
    Goal::Goal_Status stat = goal->Process(dTime);
    if(stat == eCompleted)
    //TODO: this sets defaults do check if solenoids are broken for whatever reason
        ((DoubleSolenoidItem*)(m_activeColelction->Get("hatch_push")))->DefaultSet();
    return stat;
}

void Goal_RelativeElevatorControl::Terminate()
{
    goal->Terminate();
}
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
            //cout << "no target" << endl;
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
    Log::General("vision goal active");
    m_Status = eActive;
    updateVision();
}

Goal::Goal_Status Goal_VisionAlign::Process(double dTime)
{
    m_currentTime += dTime;
    if(m_currentTime > m_timeOut)
    {
        Log::General("time out");
        Terminate();
        return m_Status = eFailed;
    }
    updateVision();
    Log::General( to_string(m_currentTarget->getX()) + " " + to_string(m_currentTarget->getY()) + " " + to_string(m_currentTarget->getRadius()) + " " + to_string(Height) + " " + to_string(Width) + to_string(HasTarget));
    if(!HasTarget) 
    {
        Log::General("no target");
        StopDrive(m_activeCollection);
        return m_Status = eActive; //!return failed for real thing or search
    }
    if(m_target->compareX(m_currentTarget) < -20) 
    {
        Log::General("turn left");
        SetDrive(m_target->compareX(m_currentTarget) * TURN_KP,-(m_target->compareX(m_currentTarget) * TURN_KP),m_activeCollection);
    }
    else if(m_target->compareX(m_currentTarget) > 20)
    {
        Log::General("turn right");
        SetDrive(m_target->compareX(m_currentTarget) * TURN_KP,-(m_target->compareX(m_currentTarget) * TURN_KP),m_activeCollection);
    }
    else
    {
        if(m_target->compareRadius(m_currentTarget) < -2 || m_target->compareRadius(m_currentTarget) > 2)
        {
            Log::General("drive");
            SetDrive(m_target->compareRadius(m_currentTarget) * STRAIGHT_KP,(m_target->compareRadius(m_currentTarget) * STRAIGHT_KP),m_activeCollection);
        }
        else
        {
            Log::General("aligned");
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
#if 0
/***********************Goal_Hatch***********************/
void Goal_Hatch::Activate()
{
    m_Status = eActive;
}
#if 0
Goal::Goal_Status Goal_Hatch::Process(double dTime)
{
    //TODO: Yeet on this
}
void Goal_Hatch::Terminate()
{
	//TODO: Yeet o this
}
#endif
#endif


/**********************REVColorSensorV3*************************/

void Goal_REVColorSensorV3::Activate()
{
    m_Status = eActive;
}

Goal::Goal_Status Goal_REVColorSensorV3:: Process(double dTime)
{
    if(CurrentT < MaxT && m_Status == eActive)
    {
         if(TargetString == Color -> Get()){
            Spinner->Set(0.1);
            return m_Status = eActive;
         }
         else if(TargetString == Color -> Get()){
             Spinner->Set(0);
            return m_Status = eCompleted;
         }
         CurrentT += dTime;
    }
    else if(m_Status == eCompleted)
    {
        Spinner->Set(0);
        return m_Status = eCompleted;
    }
    else
    {
        Spinner->Set(0);
        return m_Status = eFailed;
    }
}

void Goal_REVColorSensorV3::Terminate()
{
    m_Status = eInactive;
}

/*************************Goal_Position****************************/

void Position::Activate()
{
    Spinner->SetQuadraturePosition(0);
    Bias = 100;
    m_Status = eActive;
}
Goal::Goal_Status Position::Process(double dTime)
{
    if(CurrentT < MaxT)
    {
        CurrentT += dTime;
        if(m_Status == eActive)
        {
            double Error = Spinner->Get() - m_Calculate;
            Spinner->Set(Constrain(PIDCal(P, I, D, TotalE, Error, LastE, dTime, 0.2, 0.05, LastResult, Bias), 0, 0.2));
        }
        if(Inrange(Spinner->Get(), m_Calculate, 100))
        {
            Spinner->Set(0);
            return m_Status = eCompleted;
        }
        else
        {
            return m_Status = eActive;
        }
    }
    else
    {
        Spinner->Set(0);
        if(Inrange(Spinner->Get(), m_Calculate, 100))
        {
            return m_Status = eCompleted;
        }
        else
        {
            return m_Status = eFailed;
        }
    }
}

void Position:: Terminate()
{
    Spinner->Set(0);
    m_Status = eInactive;
}

/************************Goal_MoveForward*************************/

void Goal_MoveForward::Activate()
{
    enc0 -> Reset();
    navx = m_activeCollection->GetNavX();
	navx -> Reset();
    m_Status = eActive;
    Moving = true;
    Bias = (50);
    BiasE = ((PE * distTo) * 10);
}

Goal::Goal_Status Goal_MoveForward::Process(double dTime)
{
    if(!Done && m_Status == eActive)
    {
       if(NumberAtTarget < 100 && TimePassed < TotalTime)
    	{
            enc = (enc0->GetEncoderValue());
            currentValue = (navx->GetNavXRoll()); //get new navx angle
    		//Angle PID
	    	double Error = currentValue;
            double Result = PIDCal(P, I, D, totalE, Error, PrevE, dTime, 0.5, 0.1, Pevpower, Bias);
            //Distance PID
    		double ErrorE = distTo - enc;
            double ResultE = PIDCal(PE, IE, DE, totalEncoder, ErrorE, PrevEncoder, dTime, MaxPower, Limit, PrevEResult, BiasE, ErrorTo, distTo) * (IsNegative ? -1 : 1);
            
            //Log::General("Result Left: " + to_string(Result + ResultE) + ", Result Right: " + to_string(Result - ResultE) + ", MaxPower: " + to_string(MaxPower) + ", Encoder Pos: " + to_string(enc) + ", Encoder Target: " + to_string(distTo));

            //SetDrive(Result + ResultE, Result - ResultE, m_activeCollection);
	    	SetNeoDrive(Result + ResultE, Result - ResultE, m_activeCollection); //set drive to new powers

	    	if(Inrange(enc, RealTarget, 0.05))
            {
                Log::General("In range");
                StopNeoDrive(m_activeCollection);
		    	NumberAtTarget++;
	    	}
            TimePassed += dTime;
        }
        else if(TotalTime <= TimePassed)
        {
            Done = true;
            Moving = true;
        }
        else
        {
            Done = true;
            Moving = false;
        }
    
    }
    else
    {
        //StopDrive(m_activeCollection);
    	StopNeoDrive(m_activeCollection); //once finished, stop drive
    }
    if(!Done && Moving)
         return m_Status = eActive;
    else if(Done && !Moving)
         return m_Status = eCompleted;
    else
        return m_Status = eFailed;
    
}

void Goal_MoveForward::Terminate()
{
    m_Status = eCompleted;
    //StopDrive(m_activeCollection);
    StopNeoDrive(m_activeCollection);
    Log::General("Done Moving");
}


/*********************Goal_TurnPID**********************************/

void Goal_TurnPIDF::Activate()
{
	navx -> Reset();
    m_Status = eActive;
    Moving = true;
    Bias = ((P * RealTarget)*(1.5 * (1000/RealTarget)));
}

Goal::Goal_Status Goal_TurnPIDF::Process(double dTime)
{
    if(!Done && m_Status == eActive)
    {
       if(NumberAtTarget < 400 && TimePassed < TotalTime)
    	{
    		currentValue = (double)(navx->GetNavXRoll() - Offset); //get new navx angle
    		//Angle PIDF
	    	double Error = RealTarget - currentValue;
            double Result = PIDCal(P, I, D, totalE, Error, PrevE, dTime, MaxPower, Limit, Pevpower, Bias, ErrorTo, RealTarget) * (IsNegative ? 1 : -1);

            Log::General("Angle: " + to_string(currentValue) + ", Error: " + to_string(Error));
            SetNeoDrive(Result, Result, m_activeCollection); //set drive to new powers
            //SetDrive(Result, Result, m_activeCollection);
	    	if(Inrange(currentValue, RealTarget, 10)){
		    	NumberAtTarget++;
	    	}
            TimePassed += dTime;
        }
        else if(TotalTime <= TimePassed)
        {
            Done = true;
            Moving = true;
        }
        else
        {
            Done = true;
            Moving = false;
        }
    
    }
    else
    {
        //StopDrive(m_activeCollection);
    	StopNeoDrive(m_activeCollection); //once finished, stop drive
    }
    if(!Done && Moving)
        return m_Status = eActive;
    else if(Done && !Moving)
    {
        Log::General("Time out on Gyro");
        return m_Status = eCompleted;
    }
    else
    {
        Log::General("Time out");
        return m_Status = eFailed;
    }
}

void Goal_TurnPIDF::Terminate()
{
    m_Status = eCompleted;
    //StopDrive(m_activeCollection);
    StopNeoDrive(m_activeCollection);
    Log::General("Done Moving");
}

void AutoPath::Activate()
{
    for(int i = 0; i < sizeof(Actions) / sizeof(*Actions); i++)
    {
        AddSubgoal(new Goal_TurnPIDF(m_activeCollection, Angle[i], 0.6, 4));
        AddSubgoal(new Goal_MoveForward(m_activeCollection, Dist[i], 0.6, 4));
        if(Actions[i] != 0)
        {
            if(Actions[i] == 1)
            {//Shoot
                AddSubgoal(new Goal_ShooterBunch(m_activeCollection));
            }
            else if(Actions[i] == 2)
            {
                //intake
                //((VictorSPItem*)(m_activeCollection->Get("Intake")))->Set(-1);
            }
            else if(Actions[i] == 3)
            {
                //stop intake
                //((VictorSPItem*)(m_activeCollection->Get("Intake")))->Set(0);
            }
        }
    }
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
//#pragma region MultitaskGoals
#pragma region MultitaskGoals

#pragma endregion

void Goal_ShooterYeet::Activate()
{
    m_Status = eActive;
    ShooterMotor->SetQuadraturePosition(0);
    lastPos = (ShooterMotor->GetQuadraturePosition());
    Bias = ((P * m_Speed)*(1.5 * (100000/m_Speed)));
}

Goal::Goal_Status Goal_ShooterYeet::Process(double dTime)
{
    if (m_Status == eActive)
    {
        //TODO: Have Limelight modify m_Speed depending on distance from target
        Bias = ((P * m_Speed)*(1.5 * (100000/m_Speed)));
        if((ShooterMotor->GetQuadraturePosition()) != lastPos && !FirstRun)
        {
            double EncoderValue = (ShooterMotor->GetQuadraturePosition());
            revSpeed = (int)(EncoderValue - LastE);
            {
                double Error = (m_Speed + revSpeed);
                double Result = PIDCalculae(P, I, D, total, Error, PrevE, dTime);
                PrevE = Error;

                Log::General("Result : " + to_string(Scale(Result, SlowDownBias, (Bias))));
                double SpedSpeed = (IsNegative? Constrain(Scale(Result, SlowDownBias, (Bias)), -m_MaxSpeed, 0) : Constrain(Scale(Result, SlowDownBias, (Bias)), 0, m_MaxSpeed));
                Log::General("Result Scaled: " + to_string(SpedSpeed));
                Reached = Inrange(revSpeed, m_Speed, 200);

                SpedSpeed = (ABSValue(ABSValue(LastResult) - ABSValue(SpedSpeed)) < 0.5? SpedSpeed : LastResult);

                Log::General("Target speed: " + to_string(ActualSpeedTar) + ", Rate: " + to_string(-revSpeed));
                
                {
                    ShooterMotor -> Set(SpedSpeed);
                    ShooterMotor2 ->Set(SpedSpeed);
                }
                Log::General("Power Output: " + to_string(SpedSpeed)  + ", Error: " + to_string(Error) + ", Real Power output: " + to_string(ShooterMotor->Get()) + ", Reached: " + to_string((Reached? 1 : 0)));
                Log::General("");
                LastE = EncoderValue;
                LastSpe = revSpeed;
                LastResult = SpedSpeed;
                lastPos = (ShooterMotor->GetQuadraturePosition());
            }
        }
        else if(FirstRun)
        {
            ShooterMotor -> Set((IsNegative? -0.1 : 0.1));
            ShooterMotor2 ->Set((IsNegative? -0.1 : 0.1));
            LastResult = 0.1;
            FirstRun = false;
        }
        else
        {
            /*Log::General("Idle Speed - Real Power output: " + to_string(ShooterMotor->Get()) + ", Encoder Pos: " + to_string(ShooterMotor->GetQuadraturePosition()));
            Log::General("");*/    
        }

        return eActive;
    }
    else if (m_Status = eInactive){
        ShooterMotor -> Set(0);
        return eInactive;
    }
}

void Goal_ShooterYeet::Terminate()
{
    m_Status = eInactive;
}

void Goal_ShooterBunch::Activate()
{
    Log::Error("Start");
    ShootWheel->Activate();
    m_Status = eActive;
    Lime->SetLED(0);
}

Goal::Goal_Status Goal_ShooterBunch::Process(double dTime)
{
    if(numShots < 5 && m_Status == eActive)
    {
        Log::Error("Running");
        ShootWheel->m_Speed = 10000;
        ShootWheel->Process(dTime);
        if(ShootWheel->Reached)
        {
            numShots++;
            Valve->SetForward();
            MovingFloor->Set(Speed);
            IndexL->Set(Speed);
            IndexR->Set(Speed);
        }
        else
        {
            Valve->SetReverse();
            MovingFloor->Set(0);
            IndexL->Set(0);
            IndexR->Set(0);
        }
        return m_Status = eActive;
    }
    else
    {
        ShootWheel->m_Speed = 0;
        ShootWheel->ShooterMotor->Set(0);
        ShootWheel->ShooterMotor2->Set(0);
        Valve->SetReverse();
        MovingFloor->Set(0);
        IndexL->Set(0);
        IndexR->Set(0);
        return m_Status = eCompleted;
    }
}

void Goal_ShooterBunch::Terminate()
{
    ShootWheel->Terminate();
    m_Status = eInactive;
    Lime->SetLED(1);
    Log::Error("Stop");
}
