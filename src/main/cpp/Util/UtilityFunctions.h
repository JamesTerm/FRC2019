/****************************** Header ******************************\
Class Name: -
File Name: UtilityFunctions.h
Summary: File of static utility functions for auto use.
Project: FRC2019CPP
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylann Ruiz
Comments added by Chris Weeks
Email: cooper.ryan@centaurisoft.org, ruizdylann@gmail.com
\********************************************************************/
#pragma once

#ifndef SRC_UTIL_UTILITYFUNCTIONS_H_
#define SRC_UTIL_UTILITYFUNCTIONS_H_
/*
 * THIS CLASS IS FOR SETTING AND RETREIVING AND SENDING DATA, AND SIMPLE CALCULATIONS
 */

#include "../Config/ActiveCollection.h"
#include "LoopChecks.h"

using namespace std;
using namespace Util;
using namespace Configuration;
using namespace Components;

/**********************************DRIVE METHODS**********************/
static void SetDrive(double left, double right, ActiveCollection *activeCollection) //set left and right motor power. range: [0,1]
{
	VictorSPXItem *left_0 = (VictorSPXItem*)activeCollection->Get("left_0"); //creates pointers to motor objects. This robot has three left motors and three right motors
	VictorSPXItem *left_1 = (VictorSPXItem*)activeCollection->Get("left_1");
	VictorSPXItem *right_0 = (VictorSPXItem*)activeCollection->Get("right_0");
	VictorSPXItem *right_1 = (VictorSPXItem*)activeCollection->Get("right_1");
	

	left_0->Set(left); //sets left and right motors to desired power
	left_1->Set(left);
	right_0->Set(right);
	right_1->Set(right);
}

static void SetNeoDrive(double left, double right, ActiveCollection *activeCollection)
{
	SparkMaxItem *left_0 = (SparkMaxItem*)activeCollection->Get("left1"); //creates pointers to motor objects. This robot has three left motors and three right motors
	SparkMaxItem *left_1 = (SparkMaxItem*)activeCollection->Get("left2");
	SparkMaxItem *right_0 = (SparkMaxItem*)activeCollection->Get("right1");
	SparkMaxItem *right_1 = (SparkMaxItem*)activeCollection->Get("right2");
	

	left_0->Set(left); //sets left and right motors to desired power
	left_1->Set(left);
	right_0->Set(right);
	right_1->Set(right);
}

static void StopNeoDrive(ActiveCollection *activeCollection) //sets drive power to zero
{
	SetNeoDrive(0, 0, activeCollection);
}

static void StopDrive(ActiveCollection *activeCollection) //sets drive power to zero
{
	SetDrive(0, 0, activeCollection);
}
static void DriveWithTimer(double left, double right, double sec, ActiveCollection *activeCollection) //drives for desired amount of time at specified power.
{																									//this method may not drive straight
	SetDrive(left, right, activeCollection);

	Wait(sec);

	StopDrive(activeCollection);
}

/***********************OPERATOR METHODS************************/

static void SetElevator(double power, ActiveCollection* activeCollection)
{
	Log::General("util " + to_string(power));
	SmartDashboard::PutNumber("util", power);
	((VictorSPXItem*)activeCollection->Get("elevator_0"))->Set(-power);
	((VictorSPXItem*)activeCollection->Get("elevator_1"))->Set(-power);
	((VictorSPXItem*)activeCollection->Get("elevator_2"))->Set(-power);
	((VictorSPXItem*)activeCollection->Get("elevator_3"))->Set(-power);
}
static void StopElevator(ActiveCollection* activeCollection)
{
	SetElevator(0, activeCollection);
}

/********************************AUTON METHODS********************************/
static void SlowStop(double left, double right, ActiveCollection *activeCollection) //sets motor power and decreases it over time until robot comes to a stop
{
	while ( (abs(right)) > .05 && (abs(left) > .05) ) //while motors still have significant power
	{
		left /= 1.02; //decrease by small amount
		right /= 1.02;

		SetDrive(left, right, activeCollection); //set drive to new power

		Wait(.005);
	}
	StopDrive(activeCollection);
}

	static double ABSValue(double val)
	{
		if (val < 0)
			val *= -1;
		return val;
	}

	static double roundValue(double val)
	{
		return round((float)val);
	}

	static bool Inrange(double a, double v, double T)
	{
		if (ABSValue(roundValue(a)) < ABSValue(v) + T && ABSValue(roundValue(a)) > ABSValue(v) - T)
		{
			return true;
		}
		return false;
	}

	static double Sign(double Value){
		if(Value < 0){
			return -1;
		}
		else{
			return 1;
		}
	}

	static double Scale(double Value, double Min, double MaxValue)
	{
		double A = (ABSValue(Value) / MaxValue) + Min;
		return A * Sign(Value);
	}

	static double Constrain(double Value, double Min, double Max)
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
	}

	static double PIDCalculae(double P, double I, double D, double& TotalError, double Error, double PrevE, double ChangeInTime)
	{
		TotalError += Error * ChangeInTime;
		return ((P * Error) + (I * TotalError) + (D * ((Error - PrevE) / ChangeInTime)));
	}

	static bool BelowMaxRate(double Val1, double Val2, double MaxRate)
	{
		return ABSValue(ABSValue(Val1) - ABSValue(Val2)) < MaxRate;
	}

/* DriveForward
 * This method uses two PID loops to drive straight and the requesred encoder distance
 * The first PID loop runs for the 60% of requested distance, using the navx to correct angle
 * the second PID loop runs for the remaining distance, at at a lower speed to improve accuracy.
 * 
 * What is a PID loop?
 * A PID loop is a feedback loop that uses an input stream to control an output stream. In our case, the NavX angle controls left/right motor speeds.
 * 	P - Proportion
 * 		outputs a value proportional to error (desired value minus expected value). 
 6 		For our code: We want to be facing at the same direction the whole time, so any deviation from that value is error.  
 9  I - Integral
 6 		The general term for integral is area under a graph. If you think of a graph of time versus error, integral would be the area between
 9 		the x-axis and the graph. Integral is used for error correction. Integral is not always neccessary, so you may see its scalar value (ki)
 * 		set to zero. 
 * 		For our code: Say we wanted to drive a specified distance and our robot was alrealy only 1 encoder tick away. The kp and kd values would not
 * 		provide enough power to push that last bit, because the values are so small and in the real world there's friction and stuff that would stop it.
 * 		The ki value, will observe error over time and, if it is high enough, provide that small boost that the robot needs.
 *  D - Derivative
 * 		Another way to think of derivative is slope. Derivate is the slope of the line tangent to a graph at a specific point.
 * 		Going back to the graph described above, derivitive will find how steep our error line is, aka the rate of change of error.
 * 		If we did not have kd, then kp would always overshoot the desired value, then start swinging back towards it, and overshoot again,
 * 		kinda like a pendulum trying to be at 0 degrees, it would always overshoot. Think of the kd value like a finger pushing against the pendulum.
 * 		When the pendulum isn't moving, the finger doesn't push at all, but the faster the pendulum moves, the more kd pushes back. Once tuned properly, kd
 * 		will stop the pendulum exactly at zero degrees, without it overshooting or pushing the pendulum back away. 
 * 		For our code: with only ki, the robot would oscillate around zero degress while driving forward and never stay there. With kd, the derivate will push against
 * 		ki to hopefully stop turning exactly at zero degrees.
 * 		derivative = current error - error prior. 
 */
static double DriveForward(double dist, double power, ActiveCollection *activeCollection, double T)
{

	NavX *navx = activeCollection->GetNavX();
	navx -> Reset();
	EncoderItem *enc0 = activeCollection->GetEncoder("enc0");
	bool IsNegative  = (dist < 0);
	cout << "initial angle = " << navx->GetAngle() << endl;

	double killTime = 10, elapsedTime = 0; //killTime is the time allowed before Turn method times out. Used in case robot is stuck or code messes up.

	double left, right = 0; //left and right motor powers, and default pwoer

	double currentValue = navx->GetAngle(); //get current navx angle

	double P = 5; //PID constants
	double I = -0.0005;
	double D = 8;
	double PE = 0.07; //PID constants
	double IE = 0.08;
	double DE = 0.0005;
	
	double F = (0.0) * ABSValue(dist);
	double Limit = 0.5;
	double MinPower = 0;
	double ChangeInTime = 0.004;
	double PrevE = 0, totalE = 0;
	double PrevEncoder = 0, totalEncoder = 0, PrevEncoderTrack = 10000;
	double enc = 0;
	double distTo = ABSValue(dist);

	double NumberAtTarget = 0;

	while((elapsedTime < killTime) && (NumberAtTarget < 400))
	{
		currentValue = navx->GetAngle(); //get new navx angle
		enc = ABSValue(enc0->Get()); //get new encoder distance
		//Angle PIDF
		double Error = 0 - currentValue;
        totalE += Error * ChangeInTime;
        double Result = ((PE * Error) + (IE * totalE)  + (DE * ((Error - PrevE) / ChangeInTime)));
		PrevE = Error;

		//Distance Traveled PIDF
		double ErrorEncoder = distTo - enc;
        totalEncoder += ErrorEncoder * ChangeInTime;
        double ResultEncoder = ((P * ErrorEncoder) + (I * totalEncoder)  + (D * ((ErrorEncoder - PrevEncoder) / ChangeInTime)) + F);
		PrevEncoder = ErrorEncoder;
		PrevEncoderTrack = ABSValue(PrevEncoder);
		if(power != 0)
		{
        	if (ABSValue(ResultEncoder) > power) 
			{
              ResultEncoder = power * Sign(ResultEncoder);
            } 
			else if (ABSValue(ResultEncoder) < MinPower) 
			{
                ResultEncoder = MinPower * Sign(ResultEncoder);
            }
	    }

		if(!IsNegative){
			if(ErrorEncoder > 0){
				if(ResultEncoder < 0){
					ResultEncoder = MinPower;
				}
			}
			else
			{
				if(ResultEncoder > 0){
					ResultEncoder = MinPower;
				}
			}
		}
		else{
			if(ErrorEncoder > 0){
				if(ResultEncoder > 0){
					ResultEncoder = MinPower;
				}
			}
			else
			{
				if(ResultEncoder < 0){
					ResultEncoder = MinPower;
				}
			}
		}

		if(Limit != 0)
		{
        	if (Result > Limit)
			{
              Result = Limit;
            }
			else if (Result < -Limit)
			{
                Result = -Limit;
            }
	    }
		
		if(!IsNegative)
		{
			left = -ResultEncoder - Result;
			right = ResultEncoder - Result;
		}
		else
		{
			left = ResultEncoder - Result;
			right = -ResultEncoder - Result;
		}
		
		SetDrive(left, right, activeCollection); //set drive to new powers

		if(Inrange(enc, dist, T)){
			NumberAtTarget++;
		}

		Wait(ChangeInTime);
		elapsedTime += ChangeInTime; //add time to elapsed
		//cout << "EncoderPos: " << to_string(enc) << "  : Encoder To: " << to_string(distTo) << "  : Result: " << to_string(ResultEncoder) << endl;
	}
	StopDrive(activeCollection); //once finished, stop drive
	Wait(.5);
	return 0;
}

static void MoveForwardPIDF(double Dist, double MaxPowerInput, ActiveCollection *activeCollection){
	double RealTarget = Dist * 89;         // 85 was working yesterday
	double MaxPower = MaxPowerInput;
	double x = 10;
	
	EncoderItem *enc0 = activeCollection->GetEncoder("enc0"); //gets encoder from active collection
	enc0 -> Reset();
	DriveForward(RealTarget, MaxPower, activeCollection, x);
	Log::General("ENC 0: " + to_string(enc0->Get()));
	Wait(0.5);
	Log::General("Final Error: " + to_string(RealTarget));
}


/*
 *
 * 	 							   POWER UP 2018		
 * 		                                                                                
 *                                     @#####@                                    
 *                                     @%&/#%#                                   
 *                  /%%%%%%%%%%%%%%%%%%@%&%%##%%%%%%%%%%%%%%%%%%%                 
 * @@@@@@@@@@@@@@@@@/@%&&&&%@%&&&@//@&&@%%@#%#&&&%&%&&&&/@@&&&@@%@@@@@@@@@@@@@@@@@@@@
 *                                    /@%&%%##\                                    
 *                                  /  @%%@#%#  \                                
 *                                /    @%&%%##    \                                
 *                              /      @%#%/@#      \                           
 *                            /        @%#%@/#        \                            
 *                          /       (/#@%(%&%#(@%       \                        
 *                        /       *%%*#@%#%@/#*@@%#@      \                       
 *                       ///#/@/,/@@(//@/#/,@@&///@/,/%@@//*\                      
 *                 @@@/#//@/@/@/@/@//@/@/@/@//(/@/@/@/#//@/@/@/@
 *
 * 			 		 BEGIN TRANSMISSION...=======================
 *				     RIP C#; Born: May 2014; Death Feb 5, 2018
 *					 The Day the Codebase died. It was a hard day
 *					 for BroncBotz, a hard day for the world. 
 *					 =========================...END TRANSMISSION
 *
 * 					                ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *					                | You will be missed OJ. -- Watson |
 * 					                ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  
 */
/*	
 *          ,
 *        _=|_		DEEP SPACE 2019
 *      _[_## ]_
 * _  +[_[_+_]P/    _    |_       ____      _=--|-~
 *  ~---\_I_I_[=\--~ ~~--[o]--==-|##==]-=-~~  o]H
 * -~ /[_[_|_]_]\\  -_  [[=]]    |====]  __  !j]H
 *   /    "|"    \      ^U-U^  - |    - ~ .~  U/~
 *  ~~--__~~~--__~~-__   H_H_    |_     --   _H_
 * -. _  ~~~#######~~~     ~~~-    ~~--  ._ - ~~-=
 *            ~~~=~~  -~~--  _     . -      _ _ -
 *
 *        ----------------------------------
 *       |        June, 20th, 1969          |
 *       |  Here Men from the Planet Earth  |
 *       |   First set Foot upon the Moon   |
 *       | We came in Peace for all Mankind |
 *        ---------------------------=apx=--
 */
/*
 *						INFINITE RECHARGE 2020
 *____														   ____
 *|\/|\_______________________________________________________/|\/|
 *|\/|_/__/__/__/__/__/__/__/__/_()_/__/__/__/__/__/__/__/__/__|\/|
 *|\/| 				   			 ||							   |\/|
 *|/\|						/##########\					   |\/|
 *|\/|      		  /#####/    ||	   \#####\				   |\/|
 *|/\|			|#####__________8||8__________#####|		   |\/|
 *|\/|							 \/							   |\/|
 *|/\|							 ||							   |\/|
 *|\/|							 ||							   |\/|
 *|/\|							+==\\						   ----				
 *|\/|							[---]\\ 0					_  ----				
 *|/\|							{/\/\_|_|					0| ----					
 *|\/|							0--0--0						|| ----					
 *|/\|												     __/||\\___.\			
 *|\/|___________________________________________________|_0__0__0_| 0____________			
 * ===============================================================================
 * 						It's over BroncBotz...
 * 						I HAVE THE HIGH GROUND.
 * 											-Any other Robot with a hang
 * 
 */
/* Turn
 * Uses a PID loop to turn the desired amount of degrees. PID input: Navx, output: motor power
 * What's a PID loop? Read the explanation above DriveForward()
 */
	
static double Turn(double target, ActiveCollection *activeCollection, double T)
{
	bool IsNegative = (target < 0);
	target = ABSValue(target);
	NavX *navx = activeCollection->GetNavX();
	
	navx->Reset(); //reset navx angle
	Wait(.25);

	cout << "initial angle = " << navx->GetAngle() << endl;

	double killTime = 2, elapsedTime = 0; //killTime is the time allowed before Turn method times out. Used in case robot is stuck or code messes up.

	double left, right = 0; //left and right motor powers, and default pwoer
	double power = 0;

	double currentValue = navx->GetAngle(); //get current navx angle

	double P = 0.06; //PID constants
	double I = 0.008;
	double D = 0.0005;
	double F = (0.09 * target);
	double Limit = 0.25;
	double MinLimit = 0.05;
	double ChangeInTime = 0.004;
	double PrevE, totalE = 0;
	
	double PerErrorTrack = 10000;
	double thres = T;

	while((PerErrorTrack > thres) && (elapsedTime < killTime))
	{
		currentValue = ABSValue(navx->GetAngle()); //get new navx angle

		double Error = target - currentValue;
        totalE += Error * ChangeInTime;
        double Result = ((P * Error) + (I * totalE)  + (D * ((Error - PrevE) / ChangeInTime)) + F);
		PrevE = Error;
		PerErrorTrack = Error;

		if(Limit != 0)
		{
        	if (ABSValue(Result) > Limit) {
              Result = Limit * Sign(Result);
            } else if (ABSValue(Result) < MinLimit) {
                Result = MinLimit * Sign(Result);
            }
	    }
		PerErrorTrack = ABSValue(PerErrorTrack);

		if(!IsNegative){
			left = power - Result;  //set left motor to desired power + output //might have to make postive
			right = power - Result; //set right motor to desired power - output (+ and - to make robot turn slightly) //might have to make negative again for auto

		}
		else if (IsNegative)
		{
			left = power + Result;  //set left motor to desired power + output //might have to make postive
			right = power + Result; //set right motor to desired power - output (+ and - to make robot turn slightly) //might have to make negative again for auto
		}
		if(target == 0)
		{
			left = 0;
			right = 0;
			Log::General("What are you doing trying to go to 0 using turn, a method that resets the value of the navx");
			PerErrorTrack = 0;
		}
		
		
		SetDrive(left, right, activeCollection); //set drive to new powers

		Wait(ChangeInTime);
		elapsedTime += ChangeInTime; //add time to elapsed
		
	}
	StopDrive(activeCollection); //once finished, stop drive
	Wait(.5);
	currentValue = navx->GetAngle(); //get final navx angle
	return currentValue;
}

static void TurnPIDF(double Target, ActiveCollection *activeCollection){	//This is used for turning NOT TURN()
	double TheTarget = 0;
	double RealTarget = 0;
	double Kill = 10;
	double TotalTimeSpent = 0;
	TheTarget = RealTarget = Target;
	TheTarget *= 0.8;
	RealTarget -= Turn(TheTarget, activeCollection, RealTarget * 0.1);
	Log::General("Error: " + to_string(RealTarget));
	double x = 10;
	while((RealTarget > 0.5) && (TotalTimeSpent < Kill))
	{	
		RealTarget -= Turn(RealTarget, activeCollection, x);
		Log::General("Error: " + to_string(RealTarget));
		x -= 1.5;
		if(x < 0.2){
			x = 0.2;
		}
		TotalTimeSpent++;
	}
	Wait(0.5);
	Log::General("Final Error: " + to_string(RealTarget));
	if(ABSValue(RealTarget) > 1){
		TurnPIDF(-RealTarget, activeCollection);
	}
}

#endif /* SRC_UTIL_UTILITYFUNCTIONS_H_ */
