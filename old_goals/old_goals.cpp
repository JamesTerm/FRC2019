void Goal_ElevatorControl::Activate()
{
    m_Status = eActive;
    error = 0;
    deriv = 0;
    integ = 0;
}


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

/***********************Goal_Turn***********************/
void Goal_Turn::Activate()
{
    Goal_Wait_ac::Activate();
    m_navx->ResetNav();
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