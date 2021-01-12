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


#pragma region FeedbackLoopGoals


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

/**************************Goal_Intake****************************/

void Goal_Intake::Activate()
{
    if(DeployIntake)
        Wrist->SetForward();
    else
        Wrist->SetReverse();
    IntakeMotor->Set(Sped);
    Floor->Set(Sped);
    IndexL->Set(Sped);
    IndexR->Set(Sped);    
    m_Status = eActive;
}

Goal::Goal_Status Goal_Intake::Process(double dTime)
{
    return m_Status = eCompleted;
}

void Goal_Intake::Terminate()
{
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
    Bias = (5000);
    BiasE = (100);
    Log::General("Distance to: " + to_string(distTo));
}

Goal::Goal_Status Goal_MoveForward::Process(double dTime)
{
    if(!Done && m_Status == eActive)
    {
       {
       if(NumberAtTarget < 1 && TimePassed < TotalTime && (distTo != 0))
    	{
            enc = (enc0->GetEncoderValue());
            currentValue = navx->GetNavXAngle(); //get new navx angle
    		//Angle PID
	    	double Error = currentValue;
            double Result = PIDCal(P, I, D, totalE, Error, PrevE, dTime, 0.5, 0.1, Pevpower, Bias);
            //Distance PID
    		double ErrorE = distTo - enc;
            double ResultE = PIDCal(PE, IE, DE, totalEncoder, ErrorE, PrevEncoder, dTime, MaxPower, Limit, PrevEResult, BiasE - (SBias - 1), ErrorTo, distTo) * (IsNegative ? -3 : 3);
            //Log::General("Error: " + to_string(ErrorE) + " | Dist: " + to_string(distTo));
	    	if(Inrange(0, ErrorE, 0.05))
            {
                NumberAtTarget++;
	    	}

            SetNeoDrive((Result + ResultE), (Result - ResultE), m_activeCollection); //set drive to new powers
            
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
    navx->Reset();
    m_Status = eActive;
    Moving = true;
    Bias = ((P * (RealTarget))*(1.5 * (1000/RealTarget)));
    FrameworkCommunication::GetInstance().SendData("Entered", 1);
    Log::General("Turn to: " + to_string(RealTarget));
}

void Goal_TurnPIDF::SetTarget(double Angle, double TotalT)
{
    IsNegative = Angle < 0;
    RealTarget = ABSValue(Angle);
    TotalTime = TotalT;
}

Goal::Goal_Status Goal_TurnPIDF::Process(double dTime)
{
    if(!Done && m_Status == eActive)
    {
       if(NumberAtTarget < 2 && TimePassed < TotalTime && (RealTarget != 0))
    	{
            double AngleNavX = (double)navx->GetNavXAngle();
    		currentValue = ABSValue(AngleNavX); //get new navx angle
    		//Angle PIDF
	    	double Error = RealTarget - currentValue;
            double Result = (0.025 * (IsNegative ? 1 : -1)) + (PIDCal(P, I, D, totalE, Error, PrevE, dTime, MaxPower, Limit, Pevpower, Bias, ErrorTo, RealTarget) * (IsNegative ? 1 : -1) * (SBias + 2));
            //Log::General("Error: " + to_string(Error));
            if(Inrange(0, Error, 3))
            {
		    	NumberAtTarget++;
	    	}
            SetNeoDrive(Result, Result, m_activeCollection); //set drive to new powers
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
        Log::General("Time out on Gyro at Angle: " + to_string(navx->GetNavXAngle()) + "| with a target of: " + to_string(RealTarget));
        return m_Status = eCompleted;
    }
    else
    {
        if(TotalTime != 0 && Attempts < 1)
        {
            Log::Error("Time out Turn at Angle: " + to_string(navx->GetNavXAngle()) + "| with a target of: " + to_string(RealTarget) + "! TRYING AGAIN");
            //RealTarget = ABSValue((RealTarget * (IsNegative ? -1 : 1)) - navx->GetNavXAngle());
            Log::Error("New Angle Target: " + to_string(RealTarget));
            NumberAtTarget = TimePassed = 0;
            SBias += 1;
            Attempts++;
            Moving = true;
            Done = false;
            return m_Status = eActive;
        }
        else if(TotalTime != 0 && Attempts >= 1)
        {
            Log::Error("Too many attempts to turn to: " + to_string(RealTarget));
            return m_Status = eFailed;
        }
        else
        {
            Log::General("Skipping Due to Angle near or at zero!");
            return m_Status = eCompleted;
        }
    }
}

void Goal_TurnPIDF::Terminate()
{
    m_Status = eCompleted;
    //StopDrive(m_activeCollection);
    StopNeoDrive(m_activeCollection);
    Log::General("Done Moving");
}


/********************Goal_CurvePath******************/

void Goal_CurvePath::GetLastData()
{
    if (Goal_CurvePath::HasData(0))
    {
        totalEncoder = Goal_CurvePath::GetData(0);
    }
    if (Goal_CurvePath::HasData(1))
    {
        PrevEncoder = Goal_CurvePath::GetData(1);
    }
    if (Goal_CurvePath::HasData(2))
    {
        PrevEResult = Goal_CurvePath::GetData(2);
    }
}

void Goal_CurvePath::SaveData()
{
    Goal_CurvePath::Setdata(0, totalEncoder);
    Goal_CurvePath::Setdata(1, PrevEncoder);
    Goal_CurvePath::Setdata(2, PrevEResult);
}

void Goal_CurvePath::Activate()
{
    Goal_CurvePath::GetLastData();

    encL -> Reset();
    encR -> Reset();
    navx->Reset();
    Log::General("Left: " + to_string((encL->GetEncoderValue())));
    Log::General("Right: " + to_string((encR->GetEncoderValue())));
    m_Status = eActive;
    Moving = true;
    AngleBias = ((P * (AngleTarget))*(1.5 * (1000/AngleTarget)));
    DistBias = ((P * DistBias) * 10);
    Dist = ((ABSValue(AngleTarget) * 3.1415) / 180) * (ABSValue(Radi)) * DistMulit;

    if(AngleTarget > 0)
    {
        LeftSpeed = (((AngleTarget) * 3.1415) / 180) * Constrain(ABSValue(Radi) + (Base / 24), 0, numeric_limits<double>::max());
        RightSpeed = (((AngleTarget) * 3.1415) / 180) * Constrain(ABSValue(Radi) - (Base / 24), 0, numeric_limits<double>::max());
    }
    else
    {
        LeftSpeed = (((AngleTarget) * 3.1415) / 180) * Constrain(ABSValue(Radi) - (Base / 24), 0, numeric_limits<double>::max());
        RightSpeed = (((AngleTarget) * 3.1415) / 180) * Constrain(ABSValue(Radi) + (Base / 24), 0, numeric_limits<double>::max());
    }
    
    if(LeftSpeed >= 1)
    {
        LeftSpeed /= 10;
    }
    if(RightSpeed >= 1)
    {
        RightSpeed /= 10;
    }

    LeftSpeed /= LeftAdd;
    RightSpeed /= RightAdd;

    if(LeftSpeed == 0)
    {
        RightSpeed *= 10;
    }
    else if(RightSpeed == 0)
    {
        LeftSpeed *= 10;
    }

    RightSpeed = ABSValue(RightSpeed);
    LeftSpeed = ABSValue(LeftSpeed);

    MinPowerL = encL->Get() * 0.01;
    MinPowerR = encR->Get() * 0.01;
    Log::General("Distance to travel: " + to_string(Dist) + " | Angle: " + to_string(AngleTarget) + " | Radius: " + to_string(Radi));
}

Goal::Goal_Status Goal_CurvePath::Process(double dTime)
{
    if(!Done && m_Status == eActive)
    {
        if(NumberAtTarget < 1 && TimePassed < TotalTime)
        {
            double DistError = (((((encL->GetEncoderValue() / 5) + ((encR->GetEncoderValue() / 5) * -1)) / 2) - Dist) / Dist);
            double SpeedCal = -PIDCal(P, I, D, totalEncoder, DistError, PrevEncoder, dTime, 1, Limit, PrevEResult, DistBias, DistError, 1);
            currentValue = navx->GetNavXAngle(); //get new navx angle
            ResultRight = (RightSpeed) * SpeedCal;
            ResultLeft = (LeftSpeed) * SpeedCal;
            //Log::General("Error Dist: " + to_string(DistError) + " | SpeedCal: " + to_string(SpeedCal) + " | Left: " + to_string(ResultLeft) + " | Right: " + to_string(ResultRight));
            if(Inrange(0, DistError, 0.09999))
            {
                //StopNeoDrive(m_activeCollection);
                NumberAtTarget++;
	    	}
            SetNeoDrive((ResultLeft), -(ResultRight), m_activeCollection);
            TimePassed += dTime;
        }
        else if(TotalTime <= TimePassed)
        {
            Done = true;
            Moving = true;
            Log::General("<!Time Out!>");
        }
        else
        {
            Done = true;
            Moving = false;
        }
    }
    if(!Done && Moving)
         return m_Status = eActive;
    else if(Done && !Moving)
    {
        StopNeoDrive(m_activeCollection);
        return m_Status = eCompleted;
    }
    else
    {
        StopNeoDrive(m_activeCollection);
        return m_Status = eFailed;
    }
}

void Goal_CurvePath::Terminate()
{
    Goal_CurvePath::SaveData();
    StopNeoDrive(m_activeCollection);
    m_Status = eCompleted;
}

/******************Goal_LimelightTrack***************/

void Goal_LimelightTrack::Activate()
{
    Lime->SetLED(0);
    Lime->SetCamMode(true);
    navx->Reset();
    m_Status = eActive;
}

Goal::Goal_Status Goal_LimelightTrack::Process(double dTime)
{
    if(CurrentTime < Time)
    {
        Lime->SetLED(0);
        Lime->SetCamMode(true);
        CurrentTime += dTime;
        if(!Lime->SeesTarget())
        {
            if(SRobo)
            {
                Log::General("Scaning for target");
                SetNeoDrive(Speed, Speed, m_activeCollection);
                return m_Status = eActive;
            }
            else
            {
                Log::General("Didnt See Target");
                Lime->SetLED(1);
                Lime->SetCamMode(false);
                return m_Status = eFailed;
            }
        }
        else
        {
            if(!StartUp)
            {
                Log::General("Found Target");
                StartUp = true;
            }
            double XOff = Lime->HorizontalOffset();
            SetNeoDrive(XOff / 100, XOff / 100, m_activeCollection);
            if(Inrange(ABSValue(XOff), 0, 3))
            {
                T->SetTarget(navx->GetAngle(), Time + 2);
                Log::General("Angle: " + to_string(navx->GetAngle()));
                Lime->SetLED(1);
                Lime->SetCamMode(false);
                return m_Status = eCompleted;
            }
            else
            {
                return m_Status = eActive;
            }
        
        }
    }
    else
    {
        Log::Error("Limelight failed time");
        Lime->SetLED(1);
        Lime->SetCamMode(false);
        return m_Status = eFailed;
    }
    
}

void Goal_LimelightTrack::Terminate()
{
    m_Status = eCompleted;
}

/*********************AutoPath-Goal******************/
void AutoPath::Activate()
{
    for(int i = 0; i < lenght; i++)
    {
        AddSubgoal(new Goal_CurvePath(m_activeCollection, 0, Angle[i], Radius[i], 0.8, 10, 10, 10, 23));
        
        if(Actions[i] != 0)
        {
            if(Actions[i] == 1)
            {
                //intake
                AddSubgoal(new Goal_Intake(m_activeCollection, 0.2, true));
            }
            else if(Actions[i] == 2)
            {
                //stop intake and bring in intake
                AddSubgoal(new Goal_Intake(m_activeCollection, 0, false));
            }
            else if(Actions[i] == 3)
            {
                //Shoot
                AddSubgoal(new Goal_ShooterBunch(m_activeCollection, 0));
            }
            else if(Actions[i] == 4)
            {
                //stop intake and NOT bring in intake
                AddSubgoal(new Goal_Intake(m_activeCollection, 0, true));
            }
            else if(Actions[i] == 5)
            {
                Goal_TurnPIDF* Tu = new Goal_TurnPIDF(m_activeCollection, 0, 0.3, 10, 1);
                AddSubgoal(new Goal_LimelightTrack(m_activeCollection, true, 0.1, Tu, 10));
                //Add Goals in here
                //*****************//
                
                //*****************//
                AddSubgoal(Tu);
            }
            //Add more cases here
        }
    }
    m_Status = eActive;
}

void AutoPath::Terminate()
{
    Log::General("Auto Finished");
    m_Status = eInactive;
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


#pragma endregion
//#pragma region MultitaskGoals
#pragma region MultitaskGoals

#pragma endregion

void Goal_ShooterYeet::Activate()
{
    m_Status = eActive;
    ShooterMotor->SetQuadraturePosition(0);
    lastPos = (ShooterMotor->GetQuadraturePosition());
}

Goal::Goal_Status Goal_ShooterYeet::Process(double dTime)
{
    if (m_Status == eActive)
    {
        //TODO: Have Limelight modify m_Speed depending on distance from target
        if(LimeLightTrack)
        {
            m_Speed = 69420;//Hehe
        }
        Bias = ((P * m_Speed) * 5);
        
        if((ShooterMotor->GetQuadraturePosition()) != lastPos && !FirstRun)
        {
            double EncoderValue = (ShooterMotor->GetQuadraturePosition());
            revSpeed = (int)(EncoderValue - LastE);
            {
                double Error = (m_Speed + revSpeed);
                double Result = PIDCalculae(P, I, D, total, Error, PrevE, dTime);
                PrevE = Error;

                Log::Error("Result : " + to_string(Scale(Result, SlowDownBias, (Bias))));
                double SpedSpeed = (IsNegative? Constrain(Scale(Result, SlowDownBias, (Bias)), -m_MaxSpeed, 0) : Constrain(Scale(Result, SlowDownBias, (Bias)), 0, m_MaxSpeed));
                Log::Error("Result Scaled: " + to_string(SpedSpeed));
                Reached = Inrange(revSpeed, m_Speed, 500);
                Shoot_DA_BOOL = Inrange(revSpeed, m_Speed, 50);

                SpedSpeed = (ABSValue(ABSValue(LastResult) - ABSValue(SpedSpeed)) < 0.5? SpedSpeed : LastResult);

                {
                    ShooterMotor -> Set(SpedSpeed);
                    ShooterMotor2 ->Set(SpedSpeed);
                }
                Log::Error("Power Output: " + to_string(SpedSpeed)  + ", Error: " + to_string(Error) + ", Real Power output: " + to_string(ShooterMotor->Get()) + ", Reached: " + to_string((Reached? 1 : 0)));
                Log::Error("");
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

        return eActive;
    }
    else if (m_Status = eInactive){
        ShooterMotor -> Set(0);
        ShooterMotor2 ->Set(0);
        return eInactive;
    }
}

void Goal_ShooterYeet::Terminate()
{
    ShooterMotor -> Set(0);
    ShooterMotor2 ->Set(0);
    Log::Error("Goal Shooter Yeet Stopped");
    m_Status = eInactive;
}

void Goal_ShooterBunch::Activate()
{
    Log::Error("Start");
    ShootWheel->Activate();
    m_Status = eActive;
    Lime->SetLED(0);
}
//TODO: Controller Overide
Goal::Goal_Status Goal_ShooterBunch::Process(double dTime)
{
    if(numShots < 5 && m_Status == eActive/* && CurrentTime < 10*/)
    {
        Log::Error("Target speed: " + to_string(ShootWheel->m_Speed) + ", Rate: " + to_string(-ShootWheel->revSpeed));
        Log::Error("Reached: " + to_string(ShootWheel->Reached) + ", Number Shots: " + to_string(numShots));
        ShootWheel->Process(dTime);
        if(ShootWheel->Reached)
        {
            Prep = true;
            if(ShootWheel->Shoot_DA_BOOL)
            {
                Shoot = true;
            }
        }
        if(Prep)
        {
            Log::General("Move Index");
            MovingFloor->Set(0);
            IndexL->Set(Speed);
            IndexR->Set(Speed);
            if(Shoot)
            {
                if(Increment)
                {
                    numShots++;
                    Increment = false;
                }
                Valve->SetForward();
                Prep = false;
                Shoot = false;
            }
        }
        else
        {
            MovingFloor->Set(0);
            IndexL->Set(0);
            IndexR->Set(0);
            Valve->SetReverse();
            Increment = true;
        }
        CurrentTime += dTime;
        return m_Status = eActive;
    }
    else
    {
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
    ShootWheel->ShooterMotor->Set(0);
    ShootWheel->ShooterMotor2->Set(0);
    Valve->SetReverse();
    MovingFloor->Set(0);
    IndexL->Set(0);
    IndexR->Set(0);
    Log::Error("Stop");
}
