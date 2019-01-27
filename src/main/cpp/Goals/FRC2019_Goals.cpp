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
        /////////////////////////Goal_Timer/////////////////////////
        void Goal_Timer::Activate()
        {
            m_Status = eActive;

        }
        Goal::Goal_Status Goal_Timer::Process(double dTime)
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
        void Goal_Timer::Terminate()
        {

        }
        /////////////////////////Goal_DriveWithTimer/////////////////////////
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

    #pragma region FeedbackLoopGoals
        /////////////////////////Goal_Turn/////////////////////////
        void Goal_Turn::Activate()
        {
            Goal_Timer::Activate();
            m_navx->Reset();
        }

        Goal::Goal_Status Goal_Turn::Process(double dTime)
        {
            if (m_Status == eActive)
            {
                m_currentTime += dTime;
                if (m_currentTime > m_timeOut)
                    return m_Status = eFailed; //set m_Status to failed and return m_Status in one line

                double lowerBound = m_target - m_freedom;
                double upperBound = m_target + m_freedom;
                double currentAngle = m_navx->GetAngle();

                propError = (m_target - currentAngle) / m_target; 
                integ += propError * dTime; //Right Riemann Sum integral
                deriv = (propError - errorPrior)/dTime; // rise/run slope
                errorPrior = propError; //set errorPrior for next process call

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

        /////////////////////////Goal_DriveStraight/////////////////////////
        void Goal_DriveStraight::Activate()
        {
            Goal_Timer::Activate();
            m_encLeft->Reset();
            m_encRight->Reset();
        }

        //Left enc = master, right enc = slave
        Goal::Goal_Status Goal_DriveStraight::Process(double dTime)
        {
            if (m_Status = eActive)
            {
                m_currentTime += dTime;
                if (m_currentTime > m_timeOut)
                    return m_Status = eFailed;
                m_distTraveled += m_encLeft->Get();
                if(m_distTraveled >= m_distTarget)
                {
                    Terminate();
                    return m_Status = eCompleted;
                }
                
                //TODO PID

                error = m_encLeft->Get() - m_encRight->Get(); //we cannot use true propError here because target is zero.
                integ += error * dTime; //Right Riemann Sum integral
                deriv = (error - errorPrior)/dTime; // rise/run slope
                errorPrior = error; //set errorPrior for next process call

                //left power never changes
                m_rightPower += bias + (error * kp) + (integ * ki) + (deriv * kd);
                SetDrive(m_leftPower,m_rightPower,m_activeCollection);
                
                m_encLeft->Reset();
                m_encRight->Reset();
            }
            else
            {
                return m_Status;
            }
            
        }

        void Goal_DriveStraight::Terminate()
        {
            StopDrive(m_activeCollection);
        }
    #pragma endregion
#pragma endregion

#pragma region CompositeGoals
    //TODO first thing tomorrow (1/27) check to see if code works in order
    /////////////////////////Goal_WaitThenDrive/////////////////////////
    void Goal_WaitThenDrive::Activate()
    {
    
        AddSubgoal(new Goal_DriveWithTimer(m_activeCollection, m_leftSpeed, m_rightSpeed, m_driveTime));
        AddSubgoal(new Goal_Timer(m_activeCollection, m_waitTime));
        m_Status = eActive;
    }
#pragma endregion

#pragma region MultitaskGoals

#pragma endregion