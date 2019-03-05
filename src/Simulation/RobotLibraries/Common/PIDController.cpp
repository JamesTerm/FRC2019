#include <math.h>
#include <queue>
#include <assert.h>
#include "../Base/Base_Includes.h"
#include "../Base/Misc.h"  //needed to define the declspec and IsZero
#include "PIDController.h"


  /***********************************************************************************************************/
 /*												LatencyFilter												*/
/***********************************************************************************************************/

LatencyFilter::LatencyFilter(double Latency) : m_Latency_s(Latency) 
{
	assert(m_Latency_s>=0);  //must have a positive value
}

double LatencyFilter::operator()(double input,double dTime_s)
{
	m_Queue.push(input);
	//We'll cheat and work with the current dTime_s to obtain a ballpark idea of how much time each entry is... this should be fine as long
	//as the slices are somewhat even... the idea here is to be simple and quick instead of complex and slow with a trade off of accuracy
	double CurrentLatency= m_Queue.size() * dTime_s;
	double value=m_Queue.front();
	while (CurrentLatency > m_Latency_s) 
	{
		value=m_Queue.front();
		m_Queue.pop();
		CurrentLatency= m_Queue.size() * dTime_s;
	};
	return value;
}

double LatencyFilter::operator()()
{
	return m_Queue.size()?m_Queue.front():0.0;
}

void LatencyFilter::SetLatency(double Latency)
{
	m_Latency_s=Latency;
}

  /***********************************************************************************************************/
 /*											LatencyPredictionFilter											*/
/***********************************************************************************************************/

LatencyPredictionFilter::LatencyPredictionFilter(double Latency) : m_Predicted(0.0),m_Prev_Input(0.0),m_Prev_Target(0.0),m_Latency_s(Latency) 
{
	assert(m_Latency_s>=0);  //must have a positive value
}

double LatencyPredictionFilter::operator()(double input,double target_point,double dTime_s)
{
	//avoid division by zero
	if (dTime_s==0.0) 
		return m_Predicted;

	const double CurrentRate= (input - m_Prev_Input) / dTime_s;
	const double TargetRate= (target_point - m_Prev_Target) / dTime_s;
	const double PredictedRate=(CurrentRate + TargetRate) * 0.5;
	m_Predicted=input + ( PredictedRate * m_Latency_s);
	m_Prev_Input=input;
	m_Prev_Target=target_point;
	return m_Predicted;
}

double LatencyPredictionFilter::operator()(double input,double dTime_s)
{
	//avoid division by zero
	if (dTime_s==0.0) 
		return m_Predicted;

	const double CurrentRate= (input - m_Prev_Input) / dTime_s;
	m_Predicted=input + ( CurrentRate * m_Latency_s);
	m_Prev_Input=input;
	return m_Predicted;
}

double LatencyPredictionFilter::operator()()
{
	return m_Predicted;  //This is the last value that was submitted
}

void LatencyPredictionFilter::SetLatency(double Latency)
{
	m_Latency_s=Latency;
}

  /***********************************************************************************************************/
 /*												KalmanFilter												*/
/***********************************************************************************************************/

void KalmanFilter::Reset()
{
	m_FirstRun=true;
    //initial values for the kalman filter
    m_x_est_last = 0.0;
    m_last = 0.0;
}

KalmanFilter::KalmanFilter(): m_Q(0.022),m_R(0.617)  //setup Q and R as the noise in the system
{
}

double KalmanFilter::operator()(double input)
{
	//For first run set the last value to the measured value
	if (m_FirstRun)
	{
		m_x_est_last=input;
		m_FirstRun=false;
	}
    //do a prediction
    double x_temp_est = m_x_est_last;
    double P_temp = m_last + m_Q;
    //calculate the Kalman gain
    double K = P_temp * (1.0/(P_temp + m_R));
    //the 'noisy' value we measured
    double z_measured = input;
    //correct
    double x_est = x_temp_est + K * (z_measured - x_temp_est); 
    double P = (1- K) * P_temp;
    
    //update our last's
    m_last = P;
    m_x_est_last = x_est;
    
	//Test for NAN
	if ((!(m_x_est_last>0.0)) && (!(m_x_est_last<0.0)))
		m_x_est_last=0;

    return x_est;
}

  /***********************************************************************************************************/
 /*												PIDController2												*/
/***********************************************************************************************************/


PIDController2::PIDController2(double p, double i, double d,bool AutoResetI,double maximumOutput,double minimumOutput,double maximumInput,
	double minimumInput,double m_tolerance,bool continuous,bool enabled) :
	m_P(p),m_I(i),m_D(d),m_maximumOutput(maximumOutput),m_minimumOutput(minimumOutput),m_maximumInput(maximumInput),m_minimumInput(minimumInput),
	m_continuous(continuous),m_enabled(enabled),m_AutoResetI(AutoResetI)
{
	m_prevError = 0;
	m_totalError = 0;
	m_tolerance = .05;

	m_result = 0;
}

PIDController2::~PIDController2()
{
}

double PIDController2::operator()(double setpoint,double input,double dTime_s)
{
	if (m_enabled)
	{
		//While it is true it forces client to use large values it is consistent and will yield much better results
		m_error = (setpoint - input) * dTime_s;  //Using dTime_s will keep the errors consistent if time is erratic
		if (m_continuous)
		{
			if (fabs(m_error) > 
				(m_maximumInput - m_minimumInput) / 2)
			{
				if (m_error > 0)
					m_error = m_error  - m_maximumInput + m_minimumInput;
				else
					m_error = m_error  +
					m_maximumInput - m_minimumInput;
			}
		}


		//If both the setpoint and input are zero then there should be no total error
		if (m_AutoResetI && IsZero(setpoint + input))
			m_totalError=0.0;

		//Note: here is the original code, which is correct but incomplete; 
		//this check really needs an else case, where if the error grows too large it would stop accumulating error and become stuck
		//on an undesirable value
		//if (((m_totalError + m_error) * m_I < m_maximumOutput) && ((m_totalError + m_error) * m_I > m_minimumOutput))
		//	m_totalError += m_error;

		double TotalErrorCheck=(m_totalError + m_error) * m_I;
		if (TotalErrorCheck < m_maximumOutput)
		{
			if (TotalErrorCheck > m_minimumOutput)
				m_totalError += m_error;
			else //less than the minimum output
			{
				//accumulate by an error which would equal the minimum output
				double MinError=(m_minimumOutput - ( m_I * m_totalError)) / m_I;
				m_totalError += MinError;
			}
		}
		else //greater than the maximum output
		{
			//accumulate by an error which would equal the maximum output
			double MaxError=(m_maximumOutput - ( m_I * m_totalError)) / m_I;
			m_totalError += MaxError;  
		}
				
		m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);
		m_prevError = m_error;
		
		if (m_result > m_maximumOutput)
			m_result = m_maximumOutput;
		else if (m_result < m_minimumOutput)
			m_result = m_minimumOutput;
	}
	return m_result;
}

void PIDController2::SetPID(double p, double i, double d)
{
	m_P = p;
	m_I = i;
	m_D = d;
}

double PIDController2::GetP()
{
	return m_P;
}

double PIDController2::GetI()
{
	return m_I;
}

double PIDController2::GetD()
{
	return m_D;
}

double PIDController2::Get()
{
	return m_result;
}

void PIDController2::SetContinuous(bool continuous)
{
	m_continuous = continuous;
}

void PIDController2::SetAutoResetI(bool AutoReset)
{
	m_AutoResetI=AutoReset;
}

void PIDController2::SetInputRange(double minimumInput, double maximumInput)
{
	m_minimumInput = minimumInput;
	m_maximumInput = maximumInput;	
}

void PIDController2::SetOutputRange(double minimumOutput, double maximumOutput)
{
	m_minimumOutput = minimumOutput;
	m_maximumOutput = maximumOutput;
}

double PIDController2::GetError()
{
	return m_error;
}

void PIDController2::SetTolerance(double percent)
{
	m_tolerance = percent;
}

bool PIDController2::OnTarget()
{
	return (fabs(m_error)<m_tolerance / 100 * (m_maximumInput - m_minimumInput));
}

void PIDController2::Enable()
{
	m_enabled = true;
}

void PIDController2::Disable()
{
	m_result=0.0;  //I cannot see a case where we would want to retain the last result during a disabled state
	m_enabled = false;
}

void PIDController2::Reset()
{
	m_prevError = 0;
	m_totalError = 0;
	m_result = 0;
}

void PIDController2::ResetI()
{
	m_totalError = 0;
}

void PIDController2::ResetI(double totalError)
{
	m_totalError = totalError;
}
