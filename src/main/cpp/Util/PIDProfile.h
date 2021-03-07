/****************************** Header ******************************\
Class Name: PIDProfile
File Name:	PIDProfile.h
Summary: Class that holds the PID values and calculates things
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
#include "ProfileData.h"

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

			PIDProfile(double P, double I, double D, double Bias)
            {
                SetP(P);
                SetI(I);
                SetD(D);
                SetBias(Bias);
            };

			PIDProfile(ProfileData* Data)
            {
                SetP(Data->Pval);
                SetI(Data->Ival);
                SetD(Data->Dval);
                SetBias(Data->Bias);
				SetMaxChange(Data->Change);
				SetMin(Data->Min);
				SetMax(Data->Max);
				SetInnerMax(Data->InnerMax);
				SetInnerMin(Data->InnerMin);
				SetThres(Data->Thres);
				Name = Data->Name;
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

			void SetMaxChange(double Change)
			{
				MaxChange = Change;
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

			void SetInnerMax(double Max)
            {
                InnerMax = Max;
            };

			void SetInnerMin(double Min)
            {
                InnerMin = Min;
            };

			void SetThres(double Thres)
            {
                this->Thres = Thres;
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
		        if (Distance(a, v) <= T)
	        	{
	        		return true;
	        	}
	        	return false;
        	};

        	bool Inrange(double Target, double Value)
        	{
		        if(Distance(Value, Target) <= Target)
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

			double CloserTo(double Val, double Min, double Max)
			{
				if(abs(abs(Val) - abs(Min)) < abs(abs(Val) - abs(Max)))
					return Min;
				return Max;
			};

			double RoundTo(double Val, int DecimalPlaces)
			{
				int Places = 1;
				for(int i = 0; i < DecimalPlaces; i++)
				{
					Places *= 10;
				}
				int Whole = (int)(Val * Places + 0.5);
				return (double)Whole / (double)Places; 
			};

			double Distance(double Val, double OtherVal)
			{
				return ABSValue(ABSValue(Val) - ABSValue(OtherVal));
			};

            double PIDCal(double P, double I, double D, double Target, double Current, double& LastResult, double& TotalError, double& PrevError, double& ErrorTo, double ChangeInTime, double MaxPower, double MinPower, double MaxChange, double Bias)
	        {
				if(abs(Current - Target) < Thres)
					return 0;
        		double Result = PIDCalculae(P, I, D, TotalError, (Current - Target), PrevError, ChangeInTime, ErrorTo, Target);
        		PrevError = (Current - Target);
        		Result = Constrain(Scale(Result, 0, (Bias)), MinPower, MaxPower);
				Result += (Sign(Result) > 0 ? InnerMax : InnerMin);
				Result = Constrain(Result, MinPower, MaxPower);
        		if(!BelowMaxRate(Result, LastResult, MaxChange) && Sign(Result) == Sign(LastResult))
	        	{
	        		Log::General("!!!!ERROR:-------------PIDCal went over max change, Change = " + to_string(ABSValue(ABSValue(Result) - ABSValue(LastResult))) + "!!!!");
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
            double GetLastErrorV() {return _PrevE;};
            double GetLastResult() {return _PrevR;};
			double GetErrorTo() {return _ErrorTo;};
            
            void SetTotalError(double val) { _TotalE = val;};
            void SetLastError(double val) { _PrevE = val;};
            void SetLastResult(double val) { _PrevR = val;};
			void SetErrorTo(double val) { _ErrorTo = val;};

			void SetBackgroundInfo(double TotalError, double LastError, double LastResult, double ErrorTo)
			{
				SetTotalError(TotalError);
				SetLastError(LastError);
				SetLastResult(LastResult);
				SetErrorTo(ErrorTo);
			};

            double Calculate(double Target, double Current, double D_Time)
            {
				return PIDCal(Pval, Ival, Dval, Target, Current, _PrevR, _TotalE, _PrevE, _ErrorTo, D_Time, MaxPower, MinPower, MaxChange, BiasV);
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
            double MaxChange = 1.5;

			double InnerMin = 0;
			double InnerMax = 0;
			double Thres = 0.01;

			double LastWheelEncoderVal = 0;
            double LastResult = 0;

			bool SpeedReached = false;

			string Name = "PID";
    };

	
}

#endif