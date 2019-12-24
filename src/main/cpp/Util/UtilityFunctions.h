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
static void DriveForward(double dist, double power, ActiveCollection *activeCollection)
{
		NavX *navx = activeCollection->GetNavX(); //gets navx from active collection
		navx->Reset(); //resets navx value
		EncoderItem *enc0 = activeCollection->GetEncoder("enc0"); //gets encoder from active collection
		enc0->Reset(); //resets encoder value
		Wait(.25);

		//cout << "initial angle = " << navx->GetAngle() << " initial enc = " << enc0->Get() << endl;

		double dist0 = dist*.6; //distance for first PID loop

		double left, right; //variables for left and right power

		double setPoint = navx->GetAngle(), currentValue = navx->GetAngle(), enc = enc0->Get(); //setPoint: init angle, currentValue: current angle, enc: encoder value

		double kp = .02, ki = 0, kd = .00005; //P, I, and D scalars. THESE ARE THE NUMBERS YOU TUNE
		double errorPrior = 0, error, integ = 0, deriv, output; //error prior: error from prior loop iteration
																//error: current error
																//(I)nteg, (D)eriv
																//output: value to be added or subtracted from motors

		//pid loop #1
		while(enc < dist0 && _IsAutononomous()) //while we are less than 60% of desired dist and still in auton
		{
			error = setPoint - currentValue; //set error value (P)
			integ = integ + error; //add error to integral (I)
			deriv = (error - errorPrior); //find current derivative (D)
			output = (kp*error) + (ki*integ) + (kd*deriv); //set output to PID times their respective scalars
			errorPrior = error; //set errorPrior for next iteration of loop

			//cout << "output = " << output << endl;

			left = power + output; //set left motor to desired power + output
			right = power + output; //set right motor to desired power - output (+ and - to make robot turn slightly)

			SetDrive(left, right, activeCollection); //setDrive to new powers

			currentValue = navx->GetAngle(); //get new angle for next iter
			enc = enc0->Get(); //get new encoder distance for next iter

			Wait(.05);
			//cout<<"angle = " << currentValue << "	left = " << left << "	right = " << right << endl;
		}

		power = .1; //run the remaining distance at low power. This improves accuracy
		while(enc < dist && _IsAutononomous()) //while encoder ticks is less than requested ticks AND while still in auton
		{
			error = setPoint - currentValue; //set error value (P)
			integ = integ + error; //add error to integral (I)
			deriv = (error - errorPrior); //find current derivative (D)
			output = (kp*error) + (ki*integ) + (kd*deriv); //output = PID * respective scalars
			errorPrior = error; //set errorPrior for next iteration of loop

			//cout << "output = " << output << endl;

			left = power + output; //set left motor to desired power + output
			right = power - output; //set right motor to desired power - output (+ and - to make robot turn slightly)

			SetDrive(left, right, activeCollection); //set drive to new powers

			currentValue = navx->GetAngle(); //get new navx angle for next iter
			enc = enc0->Get(); //get new encoder value for next iter

			Wait(.05);
			//cout<<"angle = " << currentValue << "	left = " << left << "	right = " << right << endl;
		}
		StopDrive(activeCollection); //once finsihed, stop driving
}

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

/* Turn
 * Uses a PID loop to turn the desired amount of degrees. PID input: Navx, output: motor power
 * What's a PID loop? Read the explination above DriveForward()
 */
static void Turn(double target, ActiveCollection *activeCollection)
{
	NavX *navx = activeCollection->GetNavX(); //gets Navx pointer
	navx->Reset(); //reset navx angle
	Wait(.25);

	cout << "initial angle = " << navx->GetAngle() << endl;

	double killTime = 10, elapsedTime = 0; //killTime is the time allowed before Turn method times out. Used in case robot is stuck or code messes up.

	double left, right = 0; //left and right motor powers, and default pwoer
	double power = 0.05;

	double currentValue = navx->GetAngle(); //get current navx angle

	double P = 1; //PID constants
	double I = 0.08;
	double D = 0.06;
	double F = -0.15;
	double Limit = 0.7;
	double ChangeInTime = 0.01;
	double PrevE, totalE = 0;
	
	double PerErrorTrack = 10;

	while((PerErrorTrack > 0.1) && (elapsedTime < killTime))
	{
		currentValue = navx->GetAngle(); //get new navx angle

		double Error = target - currentValue;
        totalE += Error * 0.4;
        double Result = ((P * Error) + (I * totalE)  + (D * ((Error - PrevE) / ChangeInTime)) + (F * target));
		PrevE = Error;
		PerErrorTrack = Error;

		if(Limit != 0)
		{
        	if (Result > Limit) {
              Result = Limit;
            } else if (Result < -Limit) {
                Result = -Limit;
            }
	    }

		if(PerErrorTrack < 0)
			PerErrorTrack *= -1;


		if(target > 0){
			left = power - Result;  //set left motor to desired power + output //might have to make postive
			right = power - Result; //set right motor to desired power - output (+ and - to make robot turn slightly) //might have to make negative again for auto

		}
		else if (target < 0)
		{
			left = power + Result;  //set left motor to desired power + output //might have to make postive
			right = power + Result; //set right motor to desired power - output (+ and - to make robot turn slightly) //might have to make negative again for auto
		}
		else
		{
			left = 0;
			right = 0;
			Log::General("What are you doing trying to go to 0 using turn, a method that resets the value of the navx");
		}
		
		
		SetDrive(left*0.85, right*0.85, activeCollection); //set drive to new powers

		Wait(ChangeInTime);
		elapsedTime += ChangeInTime; //add time to elapsed
		cout<<"angle = " << currentValue << "	left = " << left << "	right = " << right << endl;
		cout << "Error " << Error << endl;
	}
	StopDrive(activeCollection); //once finished, stop drive
	Wait(.25);
	currentValue = navx->GetAngle(); //get final navx angle
	cout << "final angle = " << currentValue << endl;
}
//DriveLong is exactly the same as DriveForward except dist0 is 75% of dist

#endif /* SRC_UTIL_UTILITYFUNCTIONS_H_ */
