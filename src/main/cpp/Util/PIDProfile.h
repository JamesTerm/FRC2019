/****************************** Header ******************************\
Class Name: PIDProfile
File Name:	PIDProfile.h
Summary: Class that holds the PID values
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#ifndef SRC_PIDProf_H_
#define SRC_PIDProf_H_

#include <iostream>
#include "cmath"

using namespace std;
using namespace frc;

namespace Util
{
    class PIDProfile
    {
        public:
            PIDProfile(double P, double I, double D)
            {
                SetP(P);
                SetI(I);
                SetD(D);
                SetBias(P * 100); //Defalt value has been used and gives an ok result (it depends on what this is used for i.e. drive train or shooter or turret)
            };

            void SetP(double P)
            {
                Pval = P;
            };
            void SetI(double I)
            {
                Ival = I;
            };
            void SetD(double D)
            {
                Dval = D;
            };
            void SetBias(double BiasVal)
            {
                BiasV = BiasVal;
            };

            void SetMax(double Max)
            {
                MaxPower = Max;
            };
            void SetMin(double Min)
            {
                MinPower = Min;
            };
            void SetChange(double delta)
            {
                MaxChange = delta;
            };

            void Reset()
            {
                _TotalE = 0;
                _PrevE = 0;
                _PrevR = 0;
                _ErrorTo = 0;
            };

            double ABSValue(double val)
        	{
        		if (val < 0)
	        		val *= -1;
		        return val;
	        };

	        double roundValue(double val)
        	{
		        return round((float)val);
        	};

	        bool Inrange(double a, double v, double T)
        	{
		        if (ABSValue(roundValue(a)) < ABSValue(v) + T && ABSValue(roundValue(a)) > ABSValue(v) - T)
	        	{
	        		return true;
	        	}
	        	return false;
        	};

        	bool Inrange(double Target, double Value)
        	{
		        if(Value <= Target)
	        	{
	        		return true;
        		}
        		return false;
        	};

        	double Sign(double Value){
        		if(Value < 0){
		        	return -1;
        		}
        		else{
		        	return 1;
		        }
	        };

	        double Scale(double Value, double Min, double MaxValue)
	        {
		        double A = (ABSValue(Value) / MaxValue) + Min;
		        return A * Sign(Value);
	        };

	        double Constrain(double Value, double Min, double Max)
	        {
		        if((Value) < Min)
		        {
			        return Min;// * Sign(Value);
		        }
		        else if((Value) > Max)
		        {
			        return Max;// * Sign(Value);
		        }
		        else
		        {
			        return Value;
		        }
	        };

	        double PIDCalculae(double P, double I, double D, double& TotalError, double Error, double PrevE, double ChangeInTime)
	        {
		        TotalError += Error * ChangeInTime;
		        return ((P * Error) + (I * TotalError) + (D * ((Error - PrevE) / ChangeInTime)));
	        };

	        double PIDCalculae(double P, double I, double D, double& TotalError, double Error, double PrevE, double ChangeInTime, double& ErrorTo, double Target)
	        {
		        TotalError += Error * ChangeInTime;
		        ErrorTo += (Error - Target) * ChangeInTime;
		        return ((P * Error) + (I * TotalError) + (D * (ErrorTo)));
	        };

	        bool BelowMaxRate(double Val1, double Val2, double MaxRate)
	        {
		        return ABSValue(ABSValue(Val1) - ABSValue(Val2)) < MaxRate;
	        };

	        double GetMax(double V1, double V2)
	        {
		        return (V1 > V2 ? V1 : V2);
	        };

	        double GetMin(double V1, double V2)
	        {
		        return (V1 < V2 ? V1 : V2);
	        };

            double PIDCal(double P, double I, double D, double& TotalError, double Error, double& PrevError, double ChangeInTime, double MaxPower, double MinPower, double MaxChange, double& LastResult, double Bias, double& ErrorTo, double Target)
	        {
        		double Result = PIDCalculae(P, I, D, TotalError, Error, PrevError, ChangeInTime, ErrorTo, Target);
        		PrevError = Error;
        		Result = Constrain(Scale(Result, 0, (Bias)), MinPower, MaxPower);
        		if(!BelowMaxRate(Result, LastResult, MaxChange))
	        	{
	        		Log::General("PIDCal went over max change, Change = " + to_string(ABSValue(ABSValue(Result) - ABSValue(LastResult))));
	        		Result = LastResult;
	        	}
        		LastResult = Result;
		        return Result;
        	};

            double GetP() {return Pval;};
            double GetU() {return Ival;};
            double GetD() {return Dval;};

            double GetBias() {return BiasV;};
            
            double GetTotalError() {return _TotalE;};
            double GetLastError() {return _PrevE;};
            double GetLastResult() {return _PrevR;};

            double Calculate(double Target, double Current, double D_Time)
            {
				return PIDCal(Pval, Ival, Dval, _TotalE, (Current - Target), _PrevE, D_Time, MaxPower, MinPower, MaxChange, _PrevR, BiasV, _ErrorTo, Target);
            };

			double CalSpeed(double SPEEEED, double MotorPower, double Enc, double D_Time)
			{
				if (SPEEEED != 0)
    			{
        			if (Inrange(MotorPower, 0, 0.01))
        			{
            			if (LastWheelEncoderVal == 0)
            			{
                			LastWheelEncoderVal = Enc;
            			}
            			double rate = (LastWheelEncoderVal - Enc);
            			double Error = rate - SPEEEED;

    			        double Result = Calculate(SPEEEED, Error, D_Time);
            			double Scaled = Scale(Result, 0.1, (GetP() * SPEEEED * 5));
            			double SpedSpeed = (SPEEEED < 0 ? Constrain(Scaled, -1, 0) : Constrain(Scaled, 0, 1));
            			SpedSpeed = (ABSValue(ABSValue(LastResult) - ABSValue(SpedSpeed)) < 0.5? SpedSpeed : LastResult);

						LastResult = SpedSpeed;
			            LastWheelEncoderVal = Enc;
			            
						SpeedReached = Inrange(SPEEEED, rate, 50);
			            return (SpedSpeed);
        			}
        			else
        			{
						SpeedReached = false;
            			return (SPEEEED > 0 ? 0.1 : -0.1);
        			}
    			}
    			else
    			{
					SpeedReached = false;
        			return (0);
    			}
			};

			bool ReachedSpeed() {return SpeedReached;};

            virtual ~PIDProfile(){};

        private:
            double Pval = 0;
            double Ival = 0;
            double Dval = 0;
            double BiasV = 0;

            double _TotalE = 0;
            double _PrevE = 0;
            double _PrevR = 0;
            double _ErrorTo = 0;

            double MinPower = -1;
            double MaxPower = 1;
            double MaxChange = 0.5;

			double LastWheelEncoderVal = 0;
            double LastResult = 0;

			bool SpeedReached = false;
    };
}

#endif