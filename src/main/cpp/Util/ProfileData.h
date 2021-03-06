/****************************** Header ******************************\
Class Name: ProfileData
File Name:	ProfileData.h
Summary: Class that holds the PID values
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#ifndef SRC_ProfData_H_
#define SRC_ProfData_H_

#include <iostream>
#include "cmath"

using namespace std;

namespace Util
{
class ProfileData
	{
		public:
		ProfileData(double P = 1, double I = 0, double D = 0, double MaxChange = 0.1, double Bias = 100, double Min = -1, double Max = 1, string Name = "Default")
		{
			Pval = P;
			Ival = I;
			Dval = D;
			Change = MaxChange;
			this->Bias = Bias;
			this->Name = Name;
			this->Min = Min;
			this->Max = Max;
		}
		ProfileData(double P, double I = 0, double D = 0, double MaxChange = 0.1, double Bias = 100, double InnerMin = 0, double InnerMax = 0, double Min = -1, double Max = 1, string Name = "Default")
		{
			Pval = P;
			Ival = I;
			Dval = D;
			Change = MaxChange;
			this->Bias = Bias;
			this->Name = Name;
			this->Min = Min;
			this->Max = Max;
			SetInnerMax(InnerMax);
			SetInnerMax(InnerMin);
		}
		void SetInnerMin(double Min = -0.15)
		{
			InnerMin = Min;
		}
		void SetInnerMax(double Max = 0.15)
		{
			InnerMax = Max;
		}
		double Pval = 0;
		double Ival = 0;
		double Dval = 0;
		double Bias = 1000;
		double Change = 0.1;
		double Min = -1;
		double Max = 1;
		double InnerMin = 0;
		double InnerMax = 0;
		string Name = "Default";
	};
}

#endif