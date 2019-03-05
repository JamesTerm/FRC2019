#include "../Base/Base_Includes.h"
#include <math.h>
#include <assert.h>
#include "../Base/Vec2d.h"
#include "../Base/Misc.h"
#include "../Base/Event.h"
#include "../Base/EventMap.h"
#include "../Base/Script.h"
#include "Entity_Properties.h"
#include "Physics_1D.h"
#include "Physics_2D.h"
#include "Entity2D.h"
#include "Goal.h"
#include "Ship_1D.h"
#include "Ship.h"
#include "AI_Base_Controller.h"
#include "Vehicle_Drive.h"
#include "PIDController.h"
#include "Poly.h"
#include "Robot_Control_Interface.h"
#include "Rotary_System.h"
#include "SmartDashboard.h"

#ifdef Robot_TesterCode
using namespace Robot_Tester;
using namespace GG_Framework::Base;
using namespace osg;
using namespace std;
#else
using namespace Framework::Base;
using namespace std;
#endif
  /***********************************************************************************************************************************/
 /*															Rotary_System															*/
/***********************************************************************************************************************************/

void Rotary_System::InitNetworkProperties(const Rotary_Props &props,bool AddArmAssist)
{
	switch (props.LoopState)
	{
		case Rotary_Props::eOpen:
			SmartDashboard::PutString("Loop State","open");
			break;
		case Rotary_Props::eClosed:
			SmartDashboard::PutString("Loop State","closed");
			break;
		case Rotary_Props::eNone:
			SmartDashboard::PutString("Loop State","none");
			break;
		case Rotary_Props::eClosed_ManualAssist:
			SmartDashboard::PutString("Loop State","closed2");
			break;
	}

	SmartDashboard::PutNumber("Tolerance",props.PrecisionTolerance);
	SmartDashboard::PutBoolean("ManualPositionTesting",false);
	SmartDashboard::PutBoolean("ManualInDegrees",true);

	if (!AddArmAssist)
	{
		SmartDashboard::PutNumber("velocity P",props.PID[0]);
		SmartDashboard::PutNumber("velocity I",props.PID[1]);
		SmartDashboard::PutNumber("velocity D",props.PID[2]);
		
		SmartDashboard::PutNumber("gain accel",props.InverseMaxAccel);
		SmartDashboard::PutNumber("gain decel",props.InverseMaxDecel);
	}
	else
	{
		const Rotary_Props::Rotary_Arm_GainAssist_Props &arm=props.ArmGainAssist; 

		SmartDashboard::PutNumber("Up P",arm.PID_Up[0]);
		SmartDashboard::PutNumber("Up I",arm.PID_Up[1]);
		SmartDashboard::PutNumber("Up D",arm.PID_Up[2]);

		SmartDashboard::PutBoolean("Use Up PID Only",arm.UsePID_Up_Only);
		if (!arm.UsePID_Up_Only)
		{
			SmartDashboard::PutNumber("Down P",arm.PID_Down[0]);
			SmartDashboard::PutNumber("Down I",arm.PID_Down[1]);
			SmartDashboard::PutNumber("Down D",arm.PID_Down[2]);
		}
		
		SmartDashboard::PutNumber("gain accel up",arm.InverseMaxAccel_Up);
		SmartDashboard::PutNumber("gain decel up",arm.InverseMaxDecel_Up);
		SmartDashboard::PutNumber("gain accel down",arm.InverseMaxAccel_Down);
		SmartDashboard::PutNumber("gain decel down",arm.InverseMaxDecel_Down);

		SmartDashboard::PutNumber("gravity gain voltage",arm.SlowVelocityVoltage);
		SmartDashboard::PutNumber("gravity gain velocity",arm.SlowVelocity);
		SmartDashboard::PutNumber("Tolerance Count",arm.ToleranceConsecutiveCount);
		SmartDashboard::PutNumber("Lag Predict Up",arm.VelocityPredictUp);
		SmartDashboard::PutNumber("Lag Predict Down",arm.VelocityPredictDown);

		SmartDashboard::PutNumber("pulse burst time",arm.PulseBurstTimeMs);
		SmartDashboard::PutNumber("pulse burst range",arm.PulseBurstRange);
	}
}

void Rotary_System::NetworkEditProperties(Rotary_Props &props,bool AddArmAssist)
{
	string strValue=SmartDashboard::GetString("Loop State");
	if (strcmp(strValue.c_str(),"open")==0) props.LoopState=Rotary_Props::eOpen;
	else if (strcmp(strValue.c_str(),"closed")==0) props.LoopState=Rotary_Props::eClosed;
	else if (strcmp(strValue.c_str(),"closed2")==0) props.LoopState=Rotary_Props::eClosed_ManualAssist;
	else if (strcmp(strValue.c_str(),"none")==0) props.LoopState=Rotary_Props::eNone;

	props.PrecisionTolerance=SmartDashboard::GetNumber("Tolerance");

	if (!AddArmAssist)
	{
		props.PID[0]=SmartDashboard::GetNumber("velocity P");
		props.PID[1]=SmartDashboard::GetNumber("velocity I");
		props.PID[2]=SmartDashboard::GetNumber("velocity D");

		props.InverseMaxAccel=SmartDashboard::GetNumber("gain accel");
		props.InverseMaxDecel=SmartDashboard::GetNumber("gain decel");
	}
	else
	{
		Rotary_Props::Rotary_Arm_GainAssist_Props &arm=props.ArmGainAssist; 
		arm.PID_Up[0]=SmartDashboard::GetNumber("Up P");
		arm.PID_Up[1]=SmartDashboard::GetNumber("Up I");
		arm.PID_Up[2]=SmartDashboard::GetNumber("Up D");

		bool OldCheckUpOnly=arm.UsePID_Up_Only;
		arm.UsePID_Up_Only=SmartDashboard::GetBoolean("Use Up PID Only");
		//Check for new case to populate
		if (!arm.UsePID_Up_Only && OldCheckUpOnly)
		{
			SmartDashboard::PutNumber("Down P",arm.PID_Down[0]);
			SmartDashboard::PutNumber("Down I",arm.PID_Down[1]);
			SmartDashboard::PutNumber("Down D",arm.PID_Down[2]);
		}

		if (!arm.UsePID_Up_Only)
		{
			arm.PID_Down[0]=SmartDashboard::GetNumber("Down P");
			arm.PID_Down[1]=SmartDashboard::GetNumber("Down I");
			arm.PID_Down[2]=SmartDashboard::GetNumber("Down D");
		}
		
		arm.InverseMaxAccel_Up=SmartDashboard::GetNumber("gain accel up");
		arm.InverseMaxDecel_Up=SmartDashboard::GetNumber("gain decel up");
		arm.InverseMaxAccel_Down=SmartDashboard::GetNumber("gain accel down");
		arm.InverseMaxDecel_Down=SmartDashboard::GetNumber("gain decel down");

		arm.SlowVelocityVoltage=SmartDashboard::GetNumber("gravity gain voltage");
		arm.SlowVelocity=SmartDashboard::GetNumber("gravity gain velocity");
		arm.ToleranceConsecutiveCount=SmartDashboard::GetNumber("Tolerance Count");
		arm.VelocityPredictUp=SmartDashboard::GetNumber("Lag Predict Up");
		arm.VelocityPredictDown=SmartDashboard::GetNumber("Lag Predict Down");
		arm.PulseBurstTimeMs=SmartDashboard::GetNumber("pulse burst time");
		arm.PulseBurstRange=SmartDashboard::GetNumber("pulse burst range");
	}
}

  /***********************************************************************************************************************************/
 /*														Rotary_Position_Control														*/
/***********************************************************************************************************************************/

Rotary_Position_Control::Rotary_Position_Control(const char EntityName[],Rotary_Control_Interface *robot_control,size_t InstanceIndex) : 
	Rotary_System(EntityName),m_RobotControl(robot_control),m_InstanceIndex(InstanceIndex),
	m_PIDControllerUp(0.0,0.0,0.0), //This will be overridden in properties
	m_PIDControllerDown(0.0,0.0,0.0),
	m_LastPosition(0.0),m_MatchVelocity(0.0),
	m_ErrorOffset(0.0),
	m_LastTime(0.0),m_PreviousVelocity(0.0),
	m_BurstIntensity(0.0),m_CurrentBurstTime(0.0),m_PulseBurstCounter(0),m_StallCounter(0),
	m_PotentiometerState(eNoPot), //to be safe
	m_ToleranceCounter(0)
{
}

void Rotary_Position_Control::Initialize(Base::EventMap& em,const Entity1D_Properties *props)
{
	if (m_PotentiometerState!=eNoPot)
		m_LastPosition=m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
	__super::Initialize(em,props);
	const Rotary_Properties *Props=dynamic_cast<const Rotary_Properties *>(props);
	assert(Props);
	m_VoltagePoly.Initialize(&Props->GetRotaryProps().Voltage_Terms);
	//This will copy all the props
	m_Rotary_Props=Props->GetRotaryProps();
	m_PIDControllerUp.SetPID(m_Rotary_Props.ArmGainAssist.PID_Up[0],m_Rotary_Props.ArmGainAssist.PID_Up[1],m_Rotary_Props.ArmGainAssist.PID_Up[2]);
	m_PIDControllerDown.SetPID(m_Rotary_Props.ArmGainAssist.PID_Down[0],m_Rotary_Props.ArmGainAssist.PID_Down[1],m_Rotary_Props.ArmGainAssist.PID_Down[2]);

	const double MaxSpeedReference=Props->GetMaxSpeed();
	m_PIDControllerUp.SetInputRange(-MaxSpeedReference,MaxSpeedReference);
	m_PIDControllerDown.SetInputRange(-MaxSpeedReference,MaxSpeedReference);
	double tolerance=0.99; //we must be less than one (on the positive range) to avoid lockup
	m_PIDControllerUp.SetOutputRange(-MaxSpeedReference*tolerance,MaxSpeedReference*tolerance);
	m_PIDControllerDown.SetOutputRange(-MaxSpeedReference*tolerance,MaxSpeedReference*tolerance);
	//The idea here is that the arm may rest at a stop point that needs consistent voltage to keep steady
	if (m_Rotary_Props.ArmGainAssist.SlowVelocityVoltage!=0.0)
	{
		m_PIDControllerUp.SetAutoResetI(false);
		m_PIDControllerDown.SetAutoResetI(false);
	}
	m_PIDControllerUp.Enable();
	m_PIDControllerDown.Enable();
	m_ErrorOffset=0.0;
	switch (m_Rotary_Props.LoopState)
	{
	case Rotary_Props::eNone:
		SetPotentiometerSafety(true);
		break;
	case Rotary_Props::eOpen:
		m_PotentiometerState=ePassive;
		break;
	case Rotary_Props::eClosed:
	case Rotary_Props::eClosed_ManualAssist:
		m_PotentiometerState=eActive;
		break;
	}
	//It is assumed that this property is constant throughout the whole session
	if (m_Rotary_Props.PID_Console_Dump)
	{
		Ship_1D::InitNetworkProperties(Props->GetShip_1D_Props());
		InitNetworkProperties(m_Rotary_Props,true);
	}
}

double Rotary_Position_Control::GetActualPos() const
{
	const double NewPosition=(m_PotentiometerState!=eNoPot)?m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex):GetPos_m();
	return NewPosition;
}

void Rotary_Position_Control::TimeChange(double dTime_s)
{
	//TODO we'll probably want velocity PID for turret no load type... we'll need to test to see
	const bool TuneVelocity=false;
	const double CurrentVelocity=m_Physics.GetVelocity();
	const Rotary_Props::Rotary_Arm_GainAssist_Props &arm= m_Rotary_Props.ArmGainAssist;
	const Rotary_Props::Voltage_Stall_Safety &arm_stall_safety= m_Rotary_Props.VoltageStallSafety;
	const bool NeedGainAssistForUp=((arm.SlowVelocityVoltage!=0.0)&&(CurrentVelocity>0.0));

	//Note: the order has to be in this order where it grabs the potentiometer position first and then performs the time change and finally updates the
	//new arm velocity.  Doing it this way avoids oscillating if the potentiometer and gear have been calibrated
	if (!m_LastTime) 
	{
		m_LastTime=dTime_s;
		#ifdef Robot_TesterCode
		assert(dTime_s!=0.0);
		#endif
	}

	const double NewPosition=GetActualPos();
	const double Displacement=NewPosition-m_LastPosition;
	const double PotentiometerVelocity=Displacement/m_LastTime;

	const bool UsingMoterStallSafety=(arm_stall_safety.ErrorThreshold>0.0)||(arm_stall_safety.StallCounterThreshold>0);
	double BurstIntensity=0.0;
	bool IsPulsing=false;

	//Update the position to where the potentiometer says where it actually is
	if (m_PotentiometerState==eActive)
	{
		if ((!GetLockShipToPosition()) || (m_Rotary_Props.LoopState==Rotary_Props::eClosed_ManualAssist))
		{
			if (TuneVelocity)
			{
				if (m_PIDControllerUp.GetI()==0.0)
				{
					m_ErrorOffset=m_PIDControllerUp(CurrentVelocity,PotentiometerVelocity,dTime_s);
					const double Acceleration=(CurrentVelocity-m_PreviousVelocity)/dTime_s;
					const bool Decel=(Acceleration * CurrentVelocity < 0);
					//normalize errors... these will not be reflected for I so it is safe to normalize here to avoid introducing oscillation from P
					//Note: that it is important to bias towards deceleration this can help satisfy both requirements of avoiding oscillation as well
					//As well as avoiding a potential overshoot when trying stop at a precise distance
					m_ErrorOffset=Decel || fabs(m_ErrorOffset)>m_Rotary_Props.PrecisionTolerance?m_ErrorOffset:0.0;
				}
			}
			else
			{
				const double PredictedPositionUp=NewPosition + (PotentiometerVelocity * arm.VelocityPredictUp);
				//PID will correct for position... this may need to use I to compensate for latency
				if ((arm.UsePID_Up_Only)||(GetPos_m()>PredictedPositionUp))
				{
					m_PIDControllerDown.ResetI();
					m_ErrorOffset=m_PIDControllerUp(GetPos_m(),PredictedPositionUp,dTime_s);
				}
				else
				{
					const double PredictedPosition=NewPosition + (PotentiometerVelocity * arm.VelocityPredictDown);
					m_PIDControllerUp.ResetI();
					m_ErrorOffset=m_PIDControllerDown(GetPos_m(),PredictedPosition,dTime_s);
				}

				if (UsingMoterStallSafety && (IsZero(Displacement,1e-2))&&(m_Physics.GetVelocity()>0.0))
				{
					//to measure motor stall... we evaluate the voltage
					const double MaxSpeed=m_Ship_1D_Props.MAX_SPEED;
					const double Voltage=(m_Physics.GetVelocity()+m_ErrorOffset)/MaxSpeed;
					//we may want a property for this value, but typically 0.1 is below dead zone and safe
					if (Voltage>0.1)
					{
						//printf("[%d] %d %.3f \n",m_InstanceIndex,m_StallCounter,Displacement);
						m_StallCounter++;
					}
				}
				else
					m_StallCounter=0;

				IsPulsing=((arm_stall_safety.StallCounterThreshold>0)&&(m_StallCounter>arm_stall_safety.StallCounterThreshold))||
					(arm_stall_safety.ErrorThreshold>0.0 && fabs(m_ErrorOffset)>arm_stall_safety.ErrorThreshold);

				//unlike for velocity all error offset values are taken... two PIDs and I should help stabilize oscillation
				if (((arm.PulseBurstTimeMs>0.0)&&(fabs(m_ErrorOffset)<arm.PulseBurstRange)&&(fabs(m_ErrorOffset)>m_Rotary_Props.PrecisionTolerance)) ||	(IsPulsing) )
				{
					//we are in the pulse burst zone... now to manage the burst intensity
					m_CurrentBurstTime+=dTime_s;  //increment the pulse burst time with this new time slice
					//The off-time should really match the latency time where we can sample the change interval and get a reading
					//This could be its own property if necessary
					const double Off_Time=(GetPos_m()>PredictedPositionUp)?arm.VelocityPredictUp:arm.VelocityPredictDown;
					//We use the negative sign bit to indicate it was turned off... or zero
					if (m_BurstIntensity<=0.0)
					{	//Burst has been off... is it time to turn it back on
						if (m_CurrentBurstTime>=Off_Time)
						{
							m_PulseBurstCounter++;
							if (UsingMoterStallSafety)
							{
								BurstIntensity=arm_stall_safety.OnBurstLevel;
							}
							else
							{
								//Turn on the pulse... the intensity here is computed by the overlap
								const double overlap=m_CurrentBurstTime-Off_Time;
								if (overlap<dTime_s)
									BurstIntensity=overlap/dTime_s;
								else
								{
									BurstIntensity=1.0;
									//This shouldn't happen often... probably shouldn't matter much... but keep this for diagnostic testing
									printf("test burst begin[%d]... overlap=%.2f vs delta slice=%.2f\n",m_InstanceIndex,overlap,dTime_s);
								}
							}
							m_CurrentBurstTime=0.0;  //reset timer
						}
					}
					else
					{  //Burst has been on... is it time to turn it back off
						if (m_CurrentBurstTime>=arm.PulseBurstTimeMs)
						{
							//Turn off the pulse... the intensity here is computed by the overlap
							const double overlap=m_CurrentBurstTime-arm.PulseBurstTimeMs;
							//We use the negative sign bit to indicate it was turned off... or zero
							if ((overlap<dTime_s) && (!UsingMoterStallSafety))
								BurstIntensity=-(overlap/dTime_s);
							else
							{
								BurstIntensity=0.0;
								//This shouldn't happen often... probably shouldn't matter much... but keep this for diagnostic testing
								if (!UsingMoterStallSafety)
									printf("test burst end[%d]... overlap=%.2f vs delta slice=%.2f\n",m_InstanceIndex,overlap,dTime_s);
							}
							m_CurrentBurstTime=0.0;  //reset timer
						}
						else
						{
							BurstIntensity=(!UsingMoterStallSafety)?1.0:arm_stall_safety.OnBurstLevel;  //still on under time... so keep it on full
						}
					}
				}
				//When in eClosed_ManualAssist check the locked position to end weak voltage when it is within tolerance
				//setpoint will turn locked position on once the tolerance is achieved with its count
				if (GetLockShipToPosition())
					m_ErrorOffset=fabs(m_ErrorOffset)>m_Rotary_Props.PrecisionTolerance?m_ErrorOffset:0.0;

			}
		}
		else
		{
			//If we are manually controlling, we should still update displacement to properly work with limits and maintain where the position really
			//is to seamlessly transfer between manual and auto
			m_ErrorOffset=0.0;
			m_PIDControllerUp.ResetI();
			m_PIDControllerDown.ResetI();
		}

		
		if (!IsPulsing)
			m_PulseBurstCounter=0;
		//have a way to handle to end pulsing when timeout is exceeded
		if (m_PulseBurstCounter>arm_stall_safety.PulseBurstTimeOut)
			BurstIntensity=0.0;
		//We do not want to alter position if we are using position control PID
		if ((TuneVelocity) || (IsZero(NewPosition-m_LastPosition) && (!IsPulsing)) || NeedGainAssistForUp)
			SetPos_m(NewPosition);
	}

	else if (m_PotentiometerState==ePassive)
	{
		//ensure the positions are calibrated when we are not moving
		if (IsZero(NewPosition-m_LastPosition) || NeedGainAssistForUp)
				SetPos_m(NewPosition);  //this will help min and max limits work properly even though we do not have PID
	}

	if (m_Rotary_Props.PID_Console_Dump)
	{
		const bool ManualPositionTesting=SmartDashboard::GetBoolean("ManualPositionTesting");
		const bool InDegrees=SmartDashboard::GetBoolean("ManualInDegrees");
		if (InDegrees)
		{
			if (ManualPositionTesting)
				SetIntendedPosition(DEG_2_RAD(SmartDashboard::GetNumber("IntendedPosition")));
			else
				SmartDashboard::PutNumber("IntendedPosition",RAD_2_DEG(m_IntendedPosition));
		}
		else
		{
			if (ManualPositionTesting)
				SetIntendedPosition(SmartDashboard::GetNumber("IntendedPosition"));
			else
				SmartDashboard::PutNumber("IntendedPosition",m_IntendedPosition);
		}
	}

	//if we are heading for an intended position and we graze on it... turn off the corrections
	if  (!GetLockShipToPosition())
	{
		
		if (fabs(NewPosition-m_IntendedPosition)<m_Rotary_Props.PrecisionTolerance)
			m_ToleranceCounter++;
		else
			m_ToleranceCounter=0;

		if (m_ToleranceCounter >= arm.ToleranceConsecutiveCount)
		{
			SetRequestedVelocity(0.0);  //ensure the requested velocity is zero once it gets locked to ship position
			SetCurrentLinearAcceleration(0.0);  //lock ship to position
			m_ErrorOffset=0.0;  //no error correction to factor in (avoids noisy resting point)
		}
	}

	m_LastPosition=NewPosition;
	m_LastTime=dTime_s;

	__super::TimeChange(dTime_s);

	//Note: CurrentVelocity variable is retained before the time change (for proper debugging of PID) we use the new velocity (called Velocity) here for voltage
	const double Velocity=m_Physics.GetVelocity();
	const double Acceleration=(Velocity-m_PreviousVelocity)/dTime_s;

	const double MaxSpeed=m_Ship_1D_Props.MAX_SPEED;
	double Voltage=(Velocity+m_ErrorOffset)/MaxSpeed;

	bool IsAccel=(Acceleration * Velocity > 0);
	if (Velocity>0)
		Voltage+=Acceleration*(IsAccel? arm.InverseMaxAccel_Up : arm.InverseMaxDecel_Up);
	else
		Voltage+=Acceleration*(IsAccel? arm.InverseMaxAccel_Down : arm.InverseMaxDecel_Down);

	//See if we are using the arm gain assist (only when going up)
	if ((NeedGainAssistForUp) || ((arm.SlowVelocityVoltage!=0.0) && m_ErrorOffset>0.0))
	{
		//first start out by allowing the max amount to correspond to the angle of the arm... this assumes the arm zero degrees is parallel to the ground
		//90 is straight up... should work for angles below zero (e.g. hiking viking)... angles greater than 90 will be negative which is also correct
		const double MaxVoltage=cos(NewPosition * arm.GainAssistAngleScalar) * arm.SlowVelocityVoltage;
		double BlendStrength=0.0;
		const double SlowVelocity=arm.SlowVelocity;
		//Now to compute blend strength... a simple linear distribution of how much slower it is from the slow velocity
		if (MaxVoltage>0.0)
		{
			if (PotentiometerVelocity<SlowVelocity)
				BlendStrength= (SlowVelocity-PotentiometerVelocity) / SlowVelocity;
		}
		else
		{
			if (PotentiometerVelocity>-SlowVelocity)
				BlendStrength= (SlowVelocity-fabs(PotentiometerVelocity)) / SlowVelocity;
		}
		Voltage+= MaxVoltage * BlendStrength;
	}

	//apply additional pulse burst as needed
	m_BurstIntensity=BurstIntensity;
	if (!UsingMoterStallSafety)
	{
		//The intensity we use is the same as full speed with the gain assist... we may want to have own properties for these if they do not work well
		const double PulseBurst=fabs(m_BurstIntensity) * ((m_ErrorOffset>0.0)?MaxSpeed * 1.0 * arm.InverseMaxAccel_Up : (-MaxSpeed) * 1.0 * arm.InverseMaxAccel_Down);
		Voltage+= PulseBurst;
	}
	else
	{
		if (IsPulsing)
			Voltage=m_BurstIntensity;
	}

	//if (PulseBurst!=0.0)
	//	printf("pb=%.2f\n",PulseBurst);
	#if 0
	if ((arm.PulseBurstTimeMs>0.0)&&(fabs(m_ErrorOffset)<arm.PulseBurstRange)&&(fabs(m_ErrorOffset)>m_Rotary_Props.PrecisionTolerance))
	{
		if (BurstIntensity==0)
			printf("\rOff %.2f       ",m_CurrentBurstTime);
		else
			printf("\n+On=%.2f  pb=%.2f from bi=%.2f\n",m_CurrentBurstTime,PulseBurst,m_BurstIntensity);
	}
	#endif

	//Keep track of previous velocity to compute acceleration
	m_PreviousVelocity=Velocity;
	Voltage=m_VoltagePoly(Voltage);

	if (IsZero(PotentiometerVelocity) && (CurrentVelocity==0.0))
	{
		//avoid dead zone... if we are accelerating set the dead zone to the minim value... all else zero out
		if (IsAccel)
			ComputeDeadZone(Voltage,m_Rotary_Props.Positive_DeadZone,m_Rotary_Props.Negative_DeadZone);
		else
			ComputeDeadZone(Voltage,m_Rotary_Props.Positive_DeadZone,m_Rotary_Props.Negative_DeadZone,true);
	}

	{
		//Clamp range, PID (i.e. integral) controls may saturate the amount needed
		if (Voltage>0.0)
		{
			if (Voltage>1.0)
				Voltage=1.0;
		}
		else if (Voltage<0.0)
		{
			if (Voltage<-1.0)
				Voltage=-1.0;
		}
		else
			Voltage=0.0;  //is nan case
	}

	if (m_Rotary_Props.PID_Console_Dump)
	{
		NetworkEditProperties(m_Rotary_Props,true);
		Ship_1D::NetworkEditProperties(m_Ship_1D_Props);
		m_PIDControllerUp.SetPID(m_Rotary_Props.ArmGainAssist.PID_Up[0],m_Rotary_Props.ArmGainAssist.PID_Up[1],m_Rotary_Props.ArmGainAssist.PID_Up[2]);
		m_PIDControllerDown.SetPID(m_Rotary_Props.ArmGainAssist.PID_Down[0],m_Rotary_Props.ArmGainAssist.PID_Down[1],m_Rotary_Props.ArmGainAssist.PID_Down[2]);
		switch (m_Rotary_Props.LoopState)
		{
		case Rotary_Props::eNone:
			SetPotentiometerSafety(true);
			break;
		case Rotary_Props::eOpen:
			m_PotentiometerState=ePassive;
			break;
		case Rotary_Props::eClosed:
		case Rotary_Props::eClosed_ManualAssist:
			m_PotentiometerState=eActive;
			break;
		}

		#ifdef __DebugLUA__
		const double PosY=m_LastPosition * arm.GainAssistAngleScalar; //The scalar makes position more readable
		const double PredictedPosY=GetPos_m()  * arm.GainAssistAngleScalar;
		if ((fabs(PotentiometerVelocity)>0.03)||(CurrentVelocity!=0.0)||(Voltage!=0.0))
		{
			//double PosY=RAD_2_DEG(m_LastPosition * arm.GainAssistAngleScalar);
			printf("v=%.2f y=%.2f py=%.2f p=%.2f e=%.2f eo=%.2f\n",Voltage,PosY,PredictedPosY,CurrentVelocity,PotentiometerVelocity,m_ErrorOffset);
		}
		//We may want a way to pick these separately 
		#if 1
		SmartDashboard::PutNumber("voltage",Voltage);
		SmartDashboard::PutNumber("actual y",PosY);
		SmartDashboard::PutNumber("desired y",PredictedPosY);
		SmartDashboard::PutNumber("desired velocity",CurrentVelocity);
		SmartDashboard::PutNumber("actual velocity",PotentiometerVelocity);
		SmartDashboard::PutNumber("pid error offset",m_ErrorOffset);
		#endif
		#endif
	}

	//Finally check to see if limit switches have been activated
	if ((Voltage>0.0)&&DidHitMaxLimit())
	{
		Voltage=0.0;  // disallow voltage in this direction
		ResetPosition(m_Rotary_Props.MaxLimitRange);
	}
	else if ((Voltage<0.0)&&DidHitMinLimit())
	{
		Voltage=0.0;
		ResetPosition(m_Rotary_Props.MinLimitRange);
	}
	m_RobotControl->UpdateRotaryVoltage(m_InstanceIndex,Voltage);
}

void Rotary_Position_Control::ResetPosition(double Position)
{
	__super::ResetPosition(Position);  //Let the super do it stuff first

	//TODO find case where I had this duplicate call to reset rotary I may omit this... but want to find if it needs to be here
	//  [2/18/2014 James]
	//We may need this if we use Kalman filters
	//m_RobotControl->Reset_Rotary(m_InstanceIndex);

	if ((m_PotentiometerState!=eNoPot)&&(!GetBypassPos_Update()))
	{
		m_PIDControllerUp.Reset();
		m_PIDControllerDown.Reset();
		//Only reset the encoder if we are reseting the position to the starting position
		if (Position==GetStartingPosition())
			m_RobotControl->Reset_Rotary(m_InstanceIndex);
		double NewPosition=m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
		Stop();
		SetPos_m(NewPosition);
		m_LastPosition=NewPosition;
	}
	m_ToleranceCounter=0;
}


void Rotary_Position_Control::SetPotentiometerSafety(bool DisableFeedback)
{
	//printf("\r%f       ",Value);
	if (DisableFeedback)
	{
		if (m_PotentiometerState!=eNoPot)
		{
			//first disable it
			m_PotentiometerState=eNoPot;
			//Now to reset stuff
			printf("Disabling potentiometer for %s\n",GetName().c_str());
			//m_PIDController.Reset();
			ResetPos();
			//This is no longer necessary
			m_LastPosition=0.0;
			m_ErrorOffset=0.0;
			m_LastTime=0.0;
			m_Ship_1D_Props.UsingRange=false;
		}
	}
	else
	{
		if (m_PotentiometerState==eNoPot)
		{
			switch (m_Rotary_Props.LoopState)
			{
			case Rotary_Props::eNone:
				m_PotentiometerState=eNoPot;
				//This should not happen but added for completeness
				printf("Rotary_Velocity_Control::SetEncoderSafety %s set to no potentiometer\n",GetName().c_str());
				break;
			case Rotary_Props::eOpen:
				m_PotentiometerState=ePassive;
				break;
			case Rotary_Props::eClosed:
			case Rotary_Props::eClosed_ManualAssist:
				m_PotentiometerState=eActive;
				break;
			}

			//setup the initial value with the potentiometers value
			printf("Enabling potentiometer for %s\n",GetName().c_str());
			ResetPos();
			m_Ship_1D_Props.UsingRange=GetUsingRange_Props();
			m_ErrorOffset=0.0;
		}
	}
}
  /***********************************************************************************************************************************/
 /*														Rotary_Velocity_Control														*/
/***********************************************************************************************************************************/

Rotary_Velocity_Control::Rotary_Velocity_Control(const char EntityName[],Rotary_Control_Interface *robot_control,size_t InstanceIndex,EncoderUsage EncoderState) : 
	Rotary_System(EntityName),m_RobotControl(robot_control),m_InstanceIndex(InstanceIndex),
	m_PIDController(0.0,0.0,0.0), //This will be overridden in properties
	m_MatchVelocity(0.0),m_CalibratedScaler(1.0),m_ErrorOffset(0.0),
	m_MaxSpeedReference(0.0),m_EncoderVelocity(0.0),m_RequestedVelocity_Difference(0.0),
	m_EncoderState(EncoderState),m_PreviousVelocity(0.0)
{
}

void Rotary_Velocity_Control::Initialize(Base::EventMap& em,const Entity1D_Properties *props)
{
	if ((m_EncoderState==eActive)||(m_EncoderState==ePassive))
		m_EncoderVelocity=m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
	__super::Initialize(em,props);
	const Rotary_Properties *Props=dynamic_cast<const Rotary_Properties *>(props);
	assert(Props);
	m_VoltagePoly.Initialize(&Props->GetRotaryProps().Voltage_Terms);
	//This will copy all the props
	m_Rotary_Props=Props->GetRotaryProps();
	m_PIDController.SetPID(m_Rotary_Props.PID[0],m_Rotary_Props.PID[1],m_Rotary_Props.PID[2]);

	//Note: for the drive we create a large enough number that can divide out the voltage and small enough to recover quickly,
	//but this turned out to be problematic when using other angular rotary systems... therefore I am going to use the same computation
	//I do for linear, where it allows as slow to max speed as possible.
	//  [3/20/2012 Terminator]
	const double MaxSpeed_Forward=m_Ship_1D_Props.MaxSpeed_Forward;
	const double MaxSpeed_Reverse=m_Ship_1D_Props.MaxSpeed_Reverse;
	m_PIDController.SetInputRange(MaxSpeed_Reverse,MaxSpeed_Forward);
	double tolerance=0.99; //we must be less than one (on the positive range) to avoid lockup
	m_PIDController.SetOutputRange(MaxSpeed_Reverse*tolerance,MaxSpeed_Forward*tolerance);
	m_PIDController.Enable();
	m_CalibratedScaler=m_Ship_1D_Props.MAX_SPEED;
	m_ErrorOffset=0.0;

	switch (m_Rotary_Props.LoopState)
	{
	case Rotary_Props::eNone:
		SetEncoderSafety(true);
		break;
	case Rotary_Props::eOpen:
		m_EncoderState=ePassive;
		break;
	case Rotary_Props::eClosed:
		m_EncoderState=eActive;
		break;
	default:
		assert(false);
	}
	//It is assumed that this property is constant throughout the whole session
	if (m_Rotary_Props.PID_Console_Dump)
	{
		Ship_1D::InitNetworkProperties(Props->GetShip_1D_Props());
		InitNetworkProperties(m_Rotary_Props);
	}
}

void Rotary_Velocity_Control::UpdateRotaryProps(const Rotary_Props &RotaryProps)
{
	m_Rotary_Props=RotaryProps;
	m_CalibratedScaler=m_Ship_1D_Props.MAX_SPEED;
	m_PIDController.SetPID(m_Rotary_Props.PID[0],m_Rotary_Props.PID[1],m_Rotary_Props.PID[2]);
	switch (m_Rotary_Props.LoopState)
	{
	case Rotary_Props::eNone:
		SetEncoderSafety(true);
		break;
	case Rotary_Props::eOpen:
		m_EncoderState=ePassive;
		break;
	case Rotary_Props::eClosed:
		m_EncoderState=eActive;
		break;
	default:
		assert(false);
	}
}

void Rotary_Velocity_Control::TimeChange(double dTime_s)
{
	const double CurrentVelocity=m_Physics.GetVelocity();
	const double MaxSpeed=m_Ship_1D_Props.MAX_SPEED;

	double Encoder_Velocity=0.0;
	if ((m_EncoderState==eActive)||(m_EncoderState==ePassive))
	{
		Encoder_Velocity=m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);

		//Unlike linear there is no displacement measurement therefore no need to check GetLockShipToPosition()
		if (m_EncoderState==eActive)
		{
			if (!m_Rotary_Props.UseAggressiveStop)
			{
				double control=0.0;
				//only adjust calibration when both velocities are in the same direction, or in the case where the encoder is stopped which will
				//allow the scaler to normalize if it need to start up again.
				if (((CurrentVelocity * Encoder_Velocity) > 0.0) || IsZero(Encoder_Velocity) )
				{
					control=-m_PIDController(fabs(CurrentVelocity),fabs(Encoder_Velocity),dTime_s);
					m_CalibratedScaler=MaxSpeed+control;
				}
			}
			else
			{
				m_ErrorOffset=m_PIDController(CurrentVelocity,Encoder_Velocity,dTime_s);
				const double Acceleration=(CurrentVelocity-m_PreviousVelocity)/dTime_s;
				const bool Decel=(Acceleration * CurrentVelocity < 0);
				//normalize errors... these will not be reflected for I so it is safe to normalize here to avoid introducing oscillation from P
				//Note: that it is important to bias towards deceleration this can help satisfy both requirements of avoiding oscillation as well
				//As well as avoiding a potential overshoot when trying stop at a precise distance
				m_ErrorOffset=Decel || fabs(m_ErrorOffset)>m_Rotary_Props.PrecisionTolerance?m_ErrorOffset:0.0;
			}
		}
		else
			m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);  //For ease of debugging the controls (no harm to read)


		m_EncoderVelocity=Encoder_Velocity;
	}
	__super::TimeChange(dTime_s);
	const double Velocity=m_Physics.GetVelocity();
	double Acceleration=(Velocity-m_PreviousVelocity)/dTime_s;
	//CurrentVelocity is retained before the time change (for proper debugging of PID) we use the new velocity here for voltage
	//Either error offset or calibrated scaler will be used depending on the aggressive stop property, we need not branch this as
	//they both can be represented in the same equation
	double Voltage=(Velocity+m_ErrorOffset)/m_CalibratedScaler;

	bool IsAccel=(Acceleration * Velocity > 0);
	//if we are coasting we must not apply reverse voltage when decelerating
	if (!IsAccel && m_Rotary_Props.UseAggressiveStop==false)
		Acceleration=0.0;
	Voltage+=Acceleration*(IsAccel? m_Rotary_Props.InverseMaxAccel : m_Rotary_Props.InverseMaxDecel);

	//Keep track of previous velocity to compute acceleration
	m_PreviousVelocity=Velocity;

	//Apply the polynomial equation to the voltage to linearize the curve
	Voltage=m_VoltagePoly(Voltage);

	if ((IsZero(m_EncoderVelocity)) && IsAccel)
		ComputeDeadZone(Voltage,m_Rotary_Props.Positive_DeadZone,m_Rotary_Props.Negative_DeadZone);

	//Keep voltage override disabled for simulation to test precision stability
	//if (!m_VoltageOverride)
	if (true)
	{
		//Clamp range, PID (i.e. integral) controls may saturate the amount needed
		if (Voltage>0.0)
		{
			if (Voltage>1.0)
				Voltage=1.0;
		}
		else if (Voltage<0.0)
		{
			if (Voltage<-1.0)
				Voltage=-1.0;
		}
		else
			Voltage=0.0;  //is nan case
	}
	else
	{
		Voltage=0.0;
		m_PIDController.ResetI(m_MaxSpeedReference * -0.99);  //clear error for I for better transition back
	}

	#if 0
	Voltage*=Voltage;  //square them for more give
	//restore the sign
	if (CurrentVelocity<0)
		Voltage=-Voltage;
	#endif

	if (m_Rotary_Props.PID_Console_Dump)
	{
		NetworkEditProperties(m_Rotary_Props);
		Ship_1D::NetworkEditProperties(m_Ship_1D_Props);
		m_PIDController.SetPID(m_Rotary_Props.PID[0],m_Rotary_Props.PID[1],m_Rotary_Props.PID[2]);
		switch (m_Rotary_Props.LoopState)
		{
		case Rotary_Props::eNone:
			SetEncoderSafety(true);
			break;
		case Rotary_Props::eOpen:
			m_EncoderState=ePassive;
			break;
		case Rotary_Props::eClosed:
			m_EncoderState=eActive;
			break;
		default:
			assert(false);
		}

		#ifdef __DebugLUA__
		if (Encoder_Velocity!=0.0)
		{
			if (m_Rotary_Props.UseAggressiveStop)
				printf("v=%.2f p=%.2f e=%.2f eo=%.2f\n",Voltage,CurrentVelocity,Encoder_Velocity,m_ErrorOffset);
			else
			{
				if (m_PIDController.GetI()==0.0)
					printf("v=%.2f p=%.2f e=%.2f eo=%.2f cs=%.2f\n",Voltage,CurrentVelocity,Encoder_Velocity,m_ErrorOffset,m_CalibratedScaler/MaxSpeed);
				else
					printf("v=%.2f p=%.2f e=%.2f i=%.2f cs=%.2f\n",Voltage,CurrentVelocity,Encoder_Velocity,m_PIDController.GetTotalError(),m_CalibratedScaler/MaxSpeed);
			}
		}
		#endif
	}

	m_RobotControl->UpdateRotaryVoltage(m_InstanceIndex,Voltage);

}

bool Rotary_Velocity_Control::InjectDisplacement(double DeltaTime_s,double &PositionDisplacement)
{
	bool ret=false;
	const bool UpdateDisplacement=((m_EncoderState==eActive)||(m_EncoderState==ePassive));
	if (UpdateDisplacement)
	{
		double computedVelocity=m_Physics.GetVelocity();
		m_Physics.SetVelocity(m_EncoderVelocity);
		m_Physics.TimeChangeUpdate(DeltaTime_s,PositionDisplacement);
		//We must set this back so that the PID can compute the entire error
		m_Physics.SetVelocity(computedVelocity);
		ret=true;
	}
	if (!ret)
		ret=__super::InjectDisplacement(DeltaTime_s,PositionDisplacement);
	return ret;
}

void Rotary_Velocity_Control::RequestedVelocityCallback(double VelocityToUse,double DeltaTime_s)
{
	if ((m_EncoderState==eActive)||(m_EncoderState==ePassive))
		m_RequestedVelocity_Difference=VelocityToUse-m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
}

void Rotary_Velocity_Control::ResetPos()
{
	__super::ResetPos();  //Let the super do it stuff first

	m_PIDController.Reset();
	//We may need this if we use Kalman filters
	m_RobotControl->Reset_Rotary(m_InstanceIndex);
	if ((m_EncoderState==eActive)||(m_EncoderState==ePassive))
		m_EncoderVelocity=m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
	else
		m_EncoderVelocity=0.0;

	//ensure teleop has these set properly
	m_CalibratedScaler=m_Ship_1D_Props.MAX_SPEED;
	m_ErrorOffset=0.0;
	m_RequestedVelocity_Difference=0.0;
}

void Rotary_Velocity_Control::SetEncoderSafety(bool DisableFeedback)
{
	//printf("\r%f       ",Value);
	if (DisableFeedback)
	{
		if (m_EncoderState!=eNoEncoder)
		{
			//first disable it
			m_EncoderState=eNoEncoder;
			//Now to reset stuff
			printf("Disabling encoder for %s\n",GetName().c_str());
			//m_PIDController.Reset();
			ResetPos();
			//This is no longer necessary
			//m_MaxSpeed=m_MaxSpeedReference;
			m_EncoderVelocity=0.0;
			m_CalibratedScaler=m_Ship_1D_Props.MAX_SPEED;
			m_ErrorOffset=0;
			m_Ship_1D_Props.UsingRange=false;
		}
	}
	else
	{
		if (m_EncoderState==eNoEncoder)
		{
			switch (m_Rotary_Props.LoopState)
			{
			case Rotary_Props::eNone:
				m_EncoderState=eNoEncoder;
				//This should not happen but added for completeness
				printf("Rotary_Velocity_Control::SetEncoderSafety %s set to no encoder\n",GetName().c_str());
				break;
			case Rotary_Props::eOpen:
				m_EncoderState=ePassive;
				break;
			case Rotary_Props::eClosed:
				m_EncoderState=eActive;
				break;
			default:
				assert(false);
			}
			//setup the initial value with the potentiometers value
			printf("Enabling encoder for %s\n",GetName().c_str());
			ResetPos();
			m_Ship_1D_Props.UsingRange=GetUsingRange_Props();
			m_CalibratedScaler=m_Ship_1D_Props.MAX_SPEED;
			m_ErrorOffset=0;
		}
	}
}

  /***********************************************************************************************************************************/
 /*														Rotary_Properties															*/
/***********************************************************************************************************************************/

void Rotary_Properties::Init()
{
	Rotary_Props props;
	memset(&props,0,sizeof(Rotary_Props));

	props.VoltageScalar=1.0;
	props.EncoderToRS_Ratio=1.0;
	//Late assign this to override the initial default
	props.PID[0]=1.0; //set PIDs to a safe default of 1,0,0
	props.PrecisionTolerance=0.01;  //It is really hard to say what the default should be
	props.Feedback_DiplayRow=(size_t)-1;  //Only assigned to a row during calibration of feedback sensor
	props.LoopState=Rotary_Props::eNone;  //Always false when control is fully functional
	props.PID_Console_Dump=false;  //Always false unless you want to analyze PID (only one system at a time!)
	props.UseAggressiveStop=false;  //This is only for angular so false is a good default (must be explicit in script otherwise)
	props.Voltage_Terms.Init();
	props.InverseMaxAccel=props.InverseMaxDecel=0.0;
	props.Positive_DeadZone=props.Negative_DeadZone=0.0;
	props.MaxLimitRange=props.MinLimitRange=0.0;  //just zero out; these are optionally used and explicitly set when used
	props.EncoderPulsesPerRevolution=0.0;
	props.EncoderReversed_Wheel=false;
	Rotary_Props::Rotary_Arm_GainAssist_Props &arm=props.ArmGainAssist; 
	arm.SlowVelocity=arm.SlowVelocityVoltage=0.0;
	arm.GainAssistAngleScalar=1.0;
	arm.InverseMaxAccel_Down=arm.InverseMaxAccel_Up=arm.InverseMaxDecel_Down=arm.InverseMaxDecel_Up=0.0;
	for (size_t i=0;i<3;i++)
		arm.PID_Down[i]=arm.PID_Up[i]=0.0;
	arm.ToleranceConsecutiveCount=1;
	arm.VelocityPredictUp=arm.VelocityPredictDown=0.0;
	arm.UsePID_Up_Only=false;
	arm.PulseBurstRange=0.0;
	arm.PulseBurstTimeMs=0.0;
	m_RotaryProps=props;
}

void Rotary_Properties::LoadFromScript(Scripting::Script& script, bool NoDefaults)
{
	const char* err=NULL;
	Rotary_Props::Rotary_Arm_GainAssist_Props &arm=m_RotaryProps.ArmGainAssist;
	Rotary_Props::Voltage_Stall_Safety &arm_voltage_safety=m_RotaryProps.VoltageStallSafety;
	string sTest;
	//I shouldn't need this nested field redundancy... just need to be sure all client cases like this
	//err = script.GetFieldTable("rotary_settings");
	//if (!err) 
	//  [2/18/2014 James]
	//------------------------------------------------------------------------------------------------
	{
		double fValue;

		SCRIPT_INIT_DOUBLE(m_RotaryProps.VoltageScalar,"voltage_multiply");
		SCRIPT_INIT_DOUBLE(m_RotaryProps.EncoderToRS_Ratio,"encoder_to_wheel_ratio");
		SCRIPT_INIT_DOUBLE2_NoDefault(m_RotaryProps.EncoderPulsesPerRevolution,"encoder_pulses_per_revolution",360.0);

		SCRIPT_TEST_BOOL_YES(m_RotaryProps.EncoderReversed_Wheel,"encoder_reversed_wheel");
		err = script.GetFieldTable("pid");
		if (!err)
		{
			err = script.GetField("p", NULL, NULL,&m_RotaryProps.PID[0]);
			ASSERT_MSG(!err, err);
			err = script.GetField("i", NULL, NULL,&m_RotaryProps.PID[1]);
			ASSERT_MSG(!err, err);
			err = script.GetField("d", NULL, NULL,&m_RotaryProps.PID[2]);
			ASSERT_MSG(!err, err);
			script.Pop();
			for (size_t i=0;i<3;i++)
				m_RotaryProps.ArmGainAssist.PID_Up[i]=arm.PID_Down[i]=m_RotaryProps.PID[i];
		}
		err=script.GetField("use_pid_up_only", &sTest, NULL, NULL);
		if (!err)
		{
			if ((sTest.c_str()[0]=='n')||(sTest.c_str()[0]=='N')||(sTest.c_str()[0]=='0'))
				arm.UsePID_Up_Only=false;
			else if ((sTest.c_str()[0]=='y')||(sTest.c_str()[0]=='Y')||(sTest.c_str()[0]=='1'))
				arm.UsePID_Up_Only=true;
		}

		err = script.GetFieldTable("pid_up");
		if (!err)
		{
			err = script.GetField("p", NULL, NULL,&arm.PID_Up[0]);
			ASSERT_MSG(!err, err);
			err = script.GetField("i", NULL, NULL,&arm.PID_Up[1]);
			ASSERT_MSG(!err, err);
			err = script.GetField("d", NULL, NULL,&arm.PID_Up[2]);
			ASSERT_MSG(!err, err);
			script.Pop();
		}

		err = script.GetFieldTable("pid_down");
		if (!err)
		{
			err = script.GetField("p", NULL, NULL,&arm.PID_Down[0]);
			ASSERT_MSG(!err, err);
			err = script.GetField("i", NULL, NULL,&arm.PID_Down[1]);
			ASSERT_MSG(!err, err);
			err = script.GetField("d", NULL, NULL,&arm.PID_Down[2]);
			ASSERT_MSG(!err, err);
			script.Pop();
		}

		SCRIPT_INIT_DOUBLE(m_RotaryProps.PrecisionTolerance,"tolerance");

		err=script.GetField("tolerance_count", NULL, NULL, &fValue);
		if (!err)
			arm.ToleranceConsecutiveCount=fValue;

		err=script.GetField("ds_display_row", NULL, NULL, &fValue);
		if (!err)
			m_RotaryProps.Feedback_DiplayRow=(size_t)fValue;

		//I've made it closed so that typing no or NO stands out, but you can use bool as well
		//err = script.GetField("is_closed",&sTest,&bTest,NULL);
		err = script.GetField("is_closed",&sTest,NULL,NULL);
		if (!err)
		{
			if ((sTest.c_str()[0]=='n')||(sTest.c_str()[0]=='N')||(sTest.c_str()[0]=='0'))
				m_RotaryProps.LoopState=Rotary_Props::eOpen;
			else if ((sTest.c_str()[0]=='y')||(sTest.c_str()[0]=='Y')||(sTest.c_str()[0]=='1'))
				m_RotaryProps.LoopState=Rotary_Props::eClosed;
			else if (sTest.c_str()[0]=='2')		//support multiple ways to have this mode
				m_RotaryProps.LoopState=Rotary_Props::eClosed_ManualAssist;
		}
		err = script.GetField("is_closed2",&sTest,NULL,NULL);
		if (!err)
		{
			if ((sTest.c_str()[0]=='n')||(sTest.c_str()[0]=='N')||(sTest.c_str()[0]=='0'))
				m_RotaryProps.LoopState=Rotary_Props::eOpen;
			else if ((sTest.c_str()[0]=='y')||(sTest.c_str()[0]=='Y')||(sTest.c_str()[0]=='1'))
				m_RotaryProps.LoopState=Rotary_Props::eClosed_ManualAssist;
		}
		err = script.GetField("show_pid_dump",&sTest,NULL,NULL);
		if (!err)
		{
			if ((sTest.c_str()[0]=='y')||(sTest.c_str()[0]=='Y')||(sTest.c_str()[0]=='1'))
				m_RotaryProps.PID_Console_Dump=true;
		}
		err = script.GetField("use_aggressive_stop",&sTest,NULL,NULL);
		if (!err)
		{
			if ((sTest.c_str()[0]=='y')||(sTest.c_str()[0]=='Y')||(sTest.c_str()[0]=='1'))
				m_RotaryProps.UseAggressiveStop=true;
		}

		m_RotaryProps.Voltage_Terms.LoadFromScript(script,"curve_voltage");

		err=script.GetField("min_limit_deg", NULL, NULL, &fValue);
		if (!err) m_RotaryProps.MinLimitRange=DEG_2_RAD(fValue);
		else
		{
			err=script.GetField("min_limit", NULL, NULL, &fValue);
			if (!err) m_RotaryProps.MinLimitRange=fValue;
		}
		err=script.GetField("max_limit_deg", NULL, NULL, &fValue);
		if (!err) m_RotaryProps.MaxLimitRange=DEG_2_RAD(fValue);
		else
		{
			err=script.GetField("max_limit", NULL, NULL, &fValue);
			if (!err) m_RotaryProps.MaxLimitRange=fValue;
		}

		SCRIPT_INIT_DOUBLE(m_RotaryProps.InverseMaxAccel,"inv_max_accel");
		if (!NoDefaults)
		{
			m_RotaryProps.InverseMaxDecel=m_RotaryProps.InverseMaxAccel;	//set up deceleration to be the same value by default
			arm.InverseMaxAccel_Up=arm.InverseMaxAccel_Down=arm.InverseMaxDecel_Up=arm.InverseMaxDecel_Down=m_RotaryProps.InverseMaxAccel;
		}
		err=script.GetField("inv_max_decel", NULL, NULL, &m_RotaryProps.InverseMaxDecel);
		if (!err)
			arm.InverseMaxDecel_Up=arm.InverseMaxDecel_Down=m_RotaryProps.InverseMaxAccel;

		SCRIPT_INIT_DOUBLE(arm.InverseMaxAccel_Up,"inv_max_accel_up");
		SCRIPT_INIT_DOUBLE(arm.InverseMaxAccel_Down,"inv_max_accel_down");
		SCRIPT_INIT_DOUBLE(arm.InverseMaxDecel_Up,"inv_max_decel_up");
		SCRIPT_INIT_DOUBLE(arm.InverseMaxDecel_Down,"inv_max_decel_down");

		SCRIPT_INIT_DOUBLE(m_RotaryProps.Positive_DeadZone,"forward_deadzone");
		SCRIPT_INIT_DOUBLE(m_RotaryProps.Negative_DeadZone,"reverse_deadzone");

		//Ensure the negative settings are negative
		if (m_RotaryProps.Negative_DeadZone>0.0)
			m_RotaryProps.Negative_DeadZone=-m_RotaryProps.Negative_DeadZone;
		//TODO may want to swap forward in reverse settings if the voltage multiply is -1  (I'll want to test this as it happens)

		SCRIPT_INIT_DOUBLE(arm.SlowVelocityVoltage,	"slow_velocity_voltage");
		SCRIPT_INIT_DOUBLE(arm.SlowVelocity,			"slow_velocity");
		SCRIPT_INIT_DOUBLE(arm.GainAssistAngleScalar,	"slow_angle_scalar");
		SCRIPT_INIT_DOUBLE(arm.VelocityPredictUp,		"predict_up");
		SCRIPT_INIT_DOUBLE(arm.VelocityPredictDown,	"predict_down");
		SCRIPT_INIT_DOUBLE(arm.PulseBurstRange,		"pulse_burst_range");
		SCRIPT_INIT_DOUBLE(arm.PulseBurstTimeMs,		"pulse_burst_time");

		//if we are not using gain assist... go ahead and load the Voltage_Stall_Safety props as they are mutually exclusive
		if (arm.SlowVelocity==0.0)
		{
			SCRIPT_INIT_DOUBLE_NoDefault(arm.VelocityPredictUp,		"voltage_stall_safety_off_time");
			arm.VelocityPredictDown=arm.VelocityPredictUp;
			SCRIPT_INIT_DOUBLE_NoDefault(arm.PulseBurstTimeMs,		"voltage_stall_safety_on_time");
			SCRIPT_INIT_DOUBLE_NoDefault(arm_voltage_safety.ErrorThreshold, "voltage_stall_safety_error_threshold");
			SCRIPT_INIT_DOUBLE_NoDefault(arm_voltage_safety.OnBurstLevel, "voltage_stall_safety_on_burst_level");
			SCRIPT_INIT_SIZET_NoDefault(arm_voltage_safety.PulseBurstTimeOut, "voltage_stall_safety_burst_timeout");
			SCRIPT_INIT_SIZET_NoDefault(arm_voltage_safety.StallCounterThreshold, "voltage_stall_safety_stall_count");
		}

		#ifdef Robot_TesterCode
		err = script.GetFieldTable("motor_specs");
		if (!err)
		{
			m_EncoderSimulation.LoadFromScript(script);
			script.Pop();
		}
		#endif
	}
	__super::LoadFromScript(script, NoDefaults);
}


void Rotary_Pot_Properties::LoadFromScript(Scripting::Script& script, bool NoDefaults)
{
	const char* err=NULL;
	double fValue;
	err=script.GetField("pot_min_limit", NULL, NULL, &fValue);
	if (!err) m_RotaryPotProps.PotMinValue=fValue;
	err=script.GetField("pot_max_limit", NULL, NULL, &fValue);
	if (!err) m_RotaryPotProps.PotMaxValue=fValue;
	SCRIPT_INIT_DOUBLE_NoDefault(m_RotaryPotProps.PotLimitTolerance,"pot_limit_tolerance");
	SCRIPT_INIT_DOUBLE(m_RotaryPotProps.PotentiometerOffset,"pot_offset");
	std::string sTest;
	SCRIPT_TEST_BOOL_YES_NoDefault(m_RotaryPotProps.IsFlipped,"pot_range_flipped");
	m_RotaryPotProps.PotPolyTerms.LoadFromScript(script,"curve_pot");
	__super::LoadFromScript(script,NoDefaults);
}


  /***********************************************************************************************************************************/
 /*												Goal_Rotary_MoveToPosition															*/
/***********************************************************************************************************************************/

Goal_Rotary_MoveToPosition::Goal_Rotary_MoveToPosition(Rotary_Position_Control &rotary,double position,double tolerance,double MaxForwardSpeedRatio,double MaxReverseSpeedRatio) :
	m_rotary(rotary),m_Position(position),m_Tolerance(tolerance),m_MaxForwardSpeedRatio(MaxForwardSpeedRatio),m_MaxReverseSpeedRatio(MaxReverseSpeedRatio),m_Terminate(false)
{
	m_Status=eInactive;
}
Goal_Rotary_MoveToPosition::~Goal_Rotary_MoveToPosition()
{
	Terminate(); //more for completion
}

void Goal_Rotary_MoveToPosition::Activate() 
{
	m_Status=eActive;
	m_DefaultForwardSpeed=m_rotary.GetMaxSpeedForward();
	m_DefaultReverseSpeed=m_rotary.GetMaxSpeedReverse();
	m_rotary.SetMaxSpeedForward(m_MaxForwardSpeedRatio*m_DefaultForwardSpeed);
	m_rotary.SetMaxSpeedReverse(m_MaxReverseSpeedRatio*m_DefaultReverseSpeed);

	//During the activation we'll set the requested position
	m_rotary.SetIntendedPosition(m_Position);
}

Goal::Goal_Status Goal_Rotary_MoveToPosition::Process(double dTime_s)
{
	//TODO this may be an inline check
	if (m_Terminate)
	{
		if (m_Status==eActive)
			m_Status=eFailed;
		return m_Status;
	}
	ActivateIfInactive();
	if (m_Status==eActive)
	{
		if (m_rotary.GetIntendedPosition()==m_Position)
		{
			double position_delta=m_rotary.GetActualPos()-m_Position;
			//TODO check IsStuck for failed case
			//printf("\r%f        ",position_delta);
			if (fabs(position_delta)<m_Tolerance)  //When testing the arm it would idle around 0.02825
			{
				//printf("completed %f\n",position_delta);
				m_Status=eCompleted;
				m_rotary.SetRequestedVelocity(0.0);  //stop it
				//restore speeds
				m_rotary.SetMaxSpeedForward(m_DefaultForwardSpeed);
				m_rotary.SetMaxSpeedReverse(m_DefaultReverseSpeed);
			}
		}
		else
		{
			printf("Goal_Rotary_MoveToPosition failed\n");
			m_Status=eFailed;  //Some thing else took control of the ship
		}
	}
	return m_Status;
}

  /***********************************************************************************************************************************/
 /*												Goal_Rotary_MoveToRelativePosition													*/
/***********************************************************************************************************************************/

void Goal_Rotary_MoveToRelativePosition::Activate()
{
	//Construct a way point
	double current_pos=m_rotary.GetActualPos();
	double dest_position=m_Position+current_pos;
	m_Position=dest_position;  //This should be a one-time assignment
	__super::Activate();
}
