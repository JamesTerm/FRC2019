//Note: This is only for simulation only, as the robot will have real sensors
#ifdef _Win32

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


using namespace std;
const double c_OptimalAngleUp_r=DEG_2_RAD(70.0);
const double c_OptimalAngleDn_r=DEG_2_RAD(50.0);
const double c_ArmToGearRatio=72.0/28.0;
const double c_GearToArmRatio=1.0/c_ArmToGearRatio;
//const double c_PotentiometerToGearRatio=60.0/32.0;
//const double c_PotentiometerToArmRatio=c_PotentiometerToGearRatio * c_GearToArmRatio;
const double c_PotentiometerToArmRatio=36.0/54.0;
const double c_PotentiometerToGearRatio=c_PotentiometerToArmRatio * c_ArmToGearRatio;
const double c_ArmToPotentiometerRatio=1.0/c_PotentiometerToArmRatio;
const double c_GearToPotentiometer=1.0/c_PotentiometerToGearRatio;
//const double c_TestRate=3.0;
//const double c_TestRate=6.0;
//const double c_Potentiometer_TestRate=18.0;
const double c_Potentiometer_TestRate=24.0;
//const double c_Potentiometer_TestRate=48.0;
const double Polynomial[5]=
{
	//0.0,1.0   ,0.0    ,0.0   ,0.0
	0.0,2.4878,-2.2091,0.7134,0.0
};


  /***************************************************************************************************************/
 /*											Potentiometer_Tester												*/
/***************************************************************************************************************/

Potentiometer_Tester::Potentiometer_Tester() : m_PotentiometerProps(
	"Potentiometer",
	2.0,    //Mass
	0.0,   //Dimension  (this really does not matter for this, there is currently no functionality for this property, although it could impact limits)
	c_Potentiometer_TestRate,   //Max Speed
	1.0,1.0, //ACCEL, BRAKE  (These can be ignored)
	c_Potentiometer_TestRate,c_Potentiometer_TestRate,
	Ship_1D_Props::eRobotArm,
	true,	//Using the range
	DEG_2_RAD(-135.0),DEG_2_RAD(135.0)
	),Ship_1D("Potentiometer")

{
	m_Bypass=false;
	Initialize(m_DummyMap,&m_PotentiometerProps);
}

const double c_DeadZone=0.085;
const double c_MidRangeZone=.150;
const double c_MidDistance=c_MidRangeZone-c_DeadZone;
const double c_rMidDistance=1.0/c_MidDistance;
const double c_EndDistance=1.0-c_MidRangeZone;
const double c_rEndDistance=1.0/c_EndDistance;

double GetTweakedVoltage(double Voltage)
{
	double VoltMag=fabs(Voltage);
	double sign=Voltage / VoltMag;

	//simulate stresses on the robot
	if (VoltMag>0.0)
	{
		if (VoltMag<c_DeadZone)
		{
			//DebugOutput("Buzz %f\n",Voltage);   //simulate no movement but hearing the motor
			VoltMag=0.0;
		}
		else if (VoltMag<c_MidRangeZone)
			VoltMag=(VoltMag-c_DeadZone) * c_rMidDistance *0.5;
		else
			VoltMag=((VoltMag-c_MidRangeZone) * c_rEndDistance * 0.5) + 0.5;

		Voltage=VoltMag*sign;
	}
	return Voltage;
}

void Potentiometer_Tester::UpdatePotentiometerVoltage(double Voltage)
{
	Voltage=GetTweakedVoltage(Voltage);
	if (!m_Bypass)
		SetRequestedVelocity(Voltage*c_GearToPotentiometer*m_PotentiometerProps.GetMaxSpeed());
	else
		SetRequestedVelocity(0.0);
}

//TODO fix
static double AngleToHeight_m(double Angle_r)
{
	const double c_ArmToGearRatio=72.0/28.0;
	const double c_GearToArmRatio=1.0/c_ArmToGearRatio;
	const double c_ArmLength_m=1.8288;  //6 feet
	const double c_GearHeightOffset=1.397;  //55 inches

	return (sin(Angle_r*c_GearToArmRatio)*c_ArmLength_m)+c_GearHeightOffset;
}

double Potentiometer_Tester::GetPotentiometerCurrentPosition()
{
	//Note this is in native potentiometer ratios
	double Pos_m=GetPos_m();
	double height=AngleToHeight_m(Pos_m);

	//DOUT5("Pot=%f Angle=%f %fft %fin",m_Physics.GetVelocity(),RAD_2_DEG(Pos_m*c_GearToArmRatio),height*3.2808399,height*39.3700787);

	return Pos_m;
}

void Potentiometer_Tester::TimeChange()
{
	__super::TimeChange(m_Time_s);
}

  /***************************************************************************************************************/
 /*											Potentiometer_Tester2												*/
/***************************************************************************************************************/

Potentiometer_Tester2::Potentiometer_Tester2() : m_PotentiometerProps(
	"Potentiometer2",
	2.0,    //Mass
	0.0,   //Dimension  (this really does not matter for this, there is currently no functionality for this property, although it could impact limits)
	10.0,   //Max Speed
	1.0,1.0, //ACCEL, BRAKE  (These can be ignored)
	10.0,10.0,
	Ship_1D_Props::eSwivel,
	true,	//Using the range
	DEG_2_RAD(-180.0),DEG_2_RAD(180.0)
	),Ship_1D("Potentiometer2")

{
	m_SimulateOpposingForce=false;
}

void Potentiometer_Tester2::Initialize(const Ship_1D_Properties *props)
{
	if (props)
		m_PotentiometerProps=*props;
	__super::Initialize(m_DummyMap,&m_PotentiometerProps);
}

void Potentiometer_Tester2::SimulateOpposingForce(double Voltage)
{
	const double OuterDistance=3.8;
	const double CoreDistance=4.0;

	const double Pos=GetPos_m();
	const double Velocity=m_Physics.GetVelocity();
	double ForceScalar=0.0;
	if ((Pos>OuterDistance)&&(Velocity>=0.0))
	{
		//determine how much force is being applied
		const double Force=m_Physics.GetForceFromVelocity(Velocity+Velocity,m_Time_s);
		if (Pos<CoreDistance)
		{
			//dampen force depending on the current distance
			const double scale=1.0/(CoreDistance-OuterDistance);
			ForceScalar=(Pos-OuterDistance)*scale;
		}
		else
			ForceScalar=1.0;
		m_Physics.ApplyFractionalForce(-Force*ForceScalar,m_Time_s);
	}
	SetRequestedVelocity(Voltage*(1.0-ForceScalar)*m_PotentiometerProps.GetMaxSpeed());
}

void Potentiometer_Tester2::UpdatePotentiometerVoltage(double Voltage)
{
	//Voltage=GetTweakedVoltage(Voltage);
	if (!m_SimulateOpposingForce)
		SetRequestedVelocity(Voltage*m_PotentiometerProps.GetMaxSpeed());
	else
		SimulateOpposingForce(Voltage);
}

double Potentiometer_Tester2::GetPotentiometerCurrentPosition()
{
	//Note this is in native potentiometer ratios
	double Pos_m=GetPos_m();
	return Pos_m;
}

void Potentiometer_Tester2::TimeChange()
{
	__super::TimeChange(m_Time_s);
}

  /***************************************************************************************************************/
 /*												Potentiometer_Tester3											*/
/***************************************************************************************************************/

Potentiometer_Tester3::Potentiometer_Tester3(const char *EntityName) : Encoder_Simulator2(EntityName),m_InvEncoderToRS_Ratio(1.0)
{
}

void Potentiometer_Tester3::Initialize(const Ship_1D_Properties *props)
{
	__super::Initialize(props);
	const Rotary_Properties *rotary=dynamic_cast<const Rotary_Properties *>(props);
	if (rotary)
		m_InvEncoderToRS_Ratio=1.0/rotary->GetRotaryProps().EncoderToRS_Ratio;
	m_SlackedValue=GetDistance();
}

void Potentiometer_Tester3::UpdatePotentiometerVoltage(double Voltage)
{
	double Direction=Voltage<0 ? -1.0 : 1.0;
	Voltage=fabs(Voltage); //make positive
	//Apply the victor curve
	//Apply the polynomial equation to the voltage to linearize the curve
	{
		const double *c=Polynomial;
		double x2=Voltage*Voltage;
		double x3=Voltage*x2;
		double x4=x2*x2;
		Voltage = (c[4]*x4) + (c[3]*x3) + (c[2]*x2) + (c[1]*Voltage) + c[0]; 
		Voltage *= Direction;
	}
	//From this point it is a percentage (hopefully linear distribution after applying the curve) of the max force to apply... This can be
	//computed from stall torque ratings of motor with various gear reductions and so forth
	//on JVN's spread sheet torque at wheel is (WST / DWR) * 2  (for nm)  (Wheel stall torque / Drive Wheel Radius * 2 sides)
	//where stall torque is ST / GearReduction * DriveTrain efficiency
	//This is all computed in the drive train
	double TorqueToApply=m_DriveTrain.GetTorqueFromVoltage(Voltage);
	const double TorqueAbsorbed=m_DriveTrain.INV_GetTorqueFromVelocity(m_Physics.GetVelocity());
	TorqueToApply-=TorqueAbsorbed;
	m_Physics.ApplyFractionalTorque(TorqueToApply,m_Time_s,m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
	//If we have enough voltage and enough velocity the locking pin is not engaged... gravity can apply extra torque
	//if ((fabs(Voltage)>0.04)&&(fabs(m_Physics.GetVelocity())>0.05))
	if (fabs(Voltage)>0.04)
	{
		// t=Ia 
		//I=sum(m*r^2) or sum(AngularCoef*m*r^2)
		const double Pounds2Kilograms=0.453592;
		const double mass=16.27*Pounds2Kilograms;
		const double r2=1.82*1.82;  
		//point mass at radius is 1.0
		const double I=mass * r2;
		double Torque=I * 9.80665 * -0.55;  //the last scalar gets the direction correct with negative... and tones it down from surgical tubing
		double TorqueWithAngle=cos(GetDistance() * m_InvEncoderToRS_Ratio) * Torque * m_Time_s;  //gravity has most effect when the angle is zero
		//add surgical tubing simulation... this works in both directions  //1.6 seemed closest but on weaker battery, and can't hit 9 feet well
		TorqueWithAngle+=sin(GetDistance() * m_InvEncoderToRS_Ratio) * -1.5;
		//The pin can protect it against going the wrong direction... if they are opposite... saturate the max opposing direction
		if (((Torque * TorqueToApply)<0.0) && (fabs(TorqueWithAngle)>fabs(TorqueToApply)))
			TorqueWithAngle=-TorqueToApply;
		m_Physics.ApplyFractionalTorque(TorqueWithAngle,m_Time_s,m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
	}
}

double Potentiometer_Tester3::GetPotentiometerCurrentPosition()
{
	#if 1
	return m_SlackedValue;
	#else
	return GetDistance();
	#endif
}

void Potentiometer_Tester3::TimeChange()
{
	__super::TimeChange();
	double CurrentVelociy=m_Physics.GetVelocity();
	m_Slack.push(GetDistance());
	const size_t MaxLatencyCount=40;

	if (CurrentVelociy >= 0.0)  //going up or still shrink
	{
		if (m_Slack.size()> (MaxLatencyCount>>1))
			m_Slack.pop();
		if (!m_Slack.empty())
		{
			m_SlackedValue=m_Slack.front();
			m_Slack.pop();
		}
		else
			m_SlackedValue=GetDistance();
	}
	else  //going down expand
	{
		if (m_Slack.size()>=MaxLatencyCount)
			m_Slack.pop();
		m_SlackedValue=m_Slack.front();
	}
	//SmartDashboard::PutNumber("TestSlack",(double)m_Slack.size());
}

void Potentiometer_Tester3::ResetPos()
{
	__super::ResetPos();
	while (!m_Slack.empty())
		m_Slack.pop();
	m_SlackedValue=GetDistance();
}


  /***************************************************************************************************************/
 /*												Encoder_Simulator												*/
/***************************************************************************************************************/
#define ENCODER_TEST_RATE 1
#if ENCODER_TEST_RATE==0
const double c_Encoder_TestRate=16.0;
const double c_Encoder_MaxAccel=120.0;
#endif

#if ENCODER_TEST_RATE==1
const double c_Encoder_TestRate=2.916;
const double c_Encoder_MaxAccel=60.0;
#endif

#if ENCODER_TEST_RATE==2
const double c_Encoder_TestRate=2.4;
const double c_Encoder_MaxAccel=4.0;
#endif

#if ENCODER_TEST_RATE==3
const double c_Encoder_TestRate=1.1;
const double c_Encoder_MaxAccel=2.0;
#endif

Encoder_Simulator::Encoder_Simulator(const char EntityName[]) : m_Time_s(0.0),m_EncoderProps(
	EntityName,
	68.0,    //Mass
	0.0,   //Dimension  (this really does not matter for this, there is currently no functionality for this property, although it could impact limits)
	c_Encoder_TestRate,   //Max Speed
	1.0,1.0, //ACCEL, BRAKE  (These can be ignored)
	c_Encoder_MaxAccel,c_Encoder_MaxAccel,
	Ship_1D_Props::eRobotArm,
	false	//Not using the range
	),Ship_1D(EntityName),m_Latency(0.300),
	m_EncoderScalar(1.0),m_GetEncoderFirstCall(false)
{
}

void Encoder_Simulator::Initialize(const Ship_1D_Properties *props)
{
	if (props)
		m_EncoderProps=*props;
	__super::Initialize(m_DummyMap,&m_EncoderProps);
}

void Encoder_Simulator::UpdateEncoderVoltage(double Voltage)
{
	//Here is some stress to emulate a bad curved victor
	#if 0
	//This is how it would be if the motor was set to non-coast
	double VoltageMag=pow(fabs(Voltage),0.5);
	Voltage= (Voltage>0.0)?	VoltageMag:-VoltageMag;
	#endif
	SetRequestedVelocity(Voltage*m_EncoderProps.GetMaxSpeed());
	#if 0
	//For coast it is more like applying force
	double Velocity=m_Physics.GetVelocity();
	double MaxSpeed=m_EncoderProps.GetMaxSpeed();
	double Accel=Voltage*5000.0*MaxSpeed * m_Time_s;  //determine current acceleration by applying the ratio with a scalar  
	double filter=1.0 - (fabs(Velocity) / MaxSpeed);  //as there is more speed there is less torque
	//double friction=((Velocity>0.04)?-0.025 : (Velocity<-0.04)?+0.025 : 0.0 )* rTime ;  //There is a constant amount of friction opposite to current velocity
	double friction=((Velocity>0.04)?-250 : (Velocity<-0.04)?+250 : 0.0 )* m_Time_s;  //There is a constant amount of friction opposite to current velocity
	SetCurrentLinearAcceleration((Accel * filter * filter) + friction); //square the filter (matches read out better)
	#endif
}

double Encoder_Simulator::GetEncoderVelocity()
{
	#if 0
	return m_Physics.GetVelocity() * m_EncoderScalar;
	#else
	//if (!m_GetEncoderFirstCall) return m_Latency();

	double Voltage = m_Physics.GetVelocity() / m_EncoderProps.GetMaxSpeed();
	double Direction=Voltage<0 ? -1.0 : 1.0;
	Voltage=fabs(Voltage); //make positive
	//Apply the victor curve
	//Apply the polynomial equation to the voltage to linearize the curve
	{
		const double *c=Polynomial;
		double x2=Voltage*Voltage;
		double x3=Voltage*x2;
		double x4=x2*x2;
		Voltage = (c[4]*x4) + (c[3]*x3) + (c[2]*x2) + (c[1]*Voltage) + c[0]; 
		Voltage *= Direction;
	}
	double ret=Voltage * m_EncoderProps.GetMaxSpeed() * m_EncoderScalar;
	//ret=m_Latency(ret,m_Time_s);
	m_GetEncoderFirstCall=false; //weed out the repeat calls
	return ret;
	#endif
}

void Encoder_Simulator::SetReverseDirection(bool reverseDirection)
{
	m_EncoderScalar= reverseDirection? -1.0 : 1.0;
}

void Encoder_Simulator::TimeChange()
{
	m_GetEncoderFirstCall=true;
	__super::TimeChange(m_Time_s);
}
  /***************************************************************************************************************/
 /*											EncoderSimulation_Properties										*/
/***************************************************************************************************************/

const double c_OunceInchToNewton=0.00706155183333;

EncoderSimulation_Properties::EncoderSimulation_Properties()
{
	EncoderSimulation_Props props;
	memset(&props,0,sizeof(EncoderSimulation_Props));

	props.Wheel_Mass=1.5;
	props.COF_Efficiency=1.0;
	props.GearReduction=12.4158;
	props.TorqueAccelerationDampener=0.0508;
	props.DriveWheelRadius=0.0762;
	props.NoMotors=1.0;
	props.PayloadMass=200.0 * 0.453592;  //in kilograms
	props.SpeedLossConstant=0.81;
	props.DriveTrainEfficiency=0.9;


	props.motor.FreeSpeed_RPM=5310;
	props.motor.Stall_Torque_NM=343.4 * c_OunceInchToNewton;
	props.motor.Stall_Current_Amp=133.0;
	props.motor.Free_Current_Amp=2.7;

	m_EncoderSimulation_Props=props;
}

void EncoderSimulation_Properties::LoadFromScript(Framework::Scripting::Script& script)
{
	EncoderSimulation_Props &props=m_EncoderSimulation_Props;
	double test;
	const char* err=NULL;

	err = script.GetField("wheel_mass", NULL, NULL, &test);
	if (!err) props.Wheel_Mass=test;
	err = script.GetField("cof_efficiency", NULL, NULL, &test);
	if (!err) props.COF_Efficiency=test;
	err = script.GetField("gear_reduction", NULL, NULL, &test);
	if (!err) props.GearReduction=test;
	err = script.GetField("torque_on_wheel_radius", NULL, NULL, &test);
	if (!err) props.TorqueAccelerationDampener=test;
	err = script.GetField("drive_wheel_radius", NULL, NULL, &test);
	if (!err) props.DriveWheelRadius=test;
	err = script.GetField("number_of_motors", NULL, NULL, &test);
	if (!err) props.NoMotors=test;
	err = script.GetField("payload_mass", NULL, NULL, &test);
	if (!err) props.PayloadMass=test;
	err = script.GetField("speed_loss_constant", NULL, NULL, &test);
	if (!err) props.SpeedLossConstant=test;
	err = script.GetField("drive_train_efficiency", NULL, NULL, &test);
	if (!err) props.DriveTrainEfficiency=test;

	err = script.GetField("free_speed_rpm", NULL, NULL, &test);
	if (!err) props.motor.FreeSpeed_RPM=test;
	err = script.GetField("stall_torque", NULL, NULL, &test);
	if (!err) props.motor.Stall_Torque_NM=test;
	err = script.GetField("stall_current_amp", NULL, NULL, &test);
	if (!err) props.motor.Stall_Current_Amp=test;
	err = script.GetField("free_current_amp", NULL, NULL, &test);
	if (!err) props.motor.Free_Current_Amp=test;
}


  /***************************************************************************************************************/
 /*											Drive_Train_Characteristics											*/
/***************************************************************************************************************/

const double c_CIM_Amp_To_Torque_oz=(1.0/(133.0-2.7)) * 343.3;
const double c_CIM_Amp_To_Torque_nm=c_CIM_Amp_To_Torque_oz*c_OunceInchToNewton;
const double c_CIM_Vel_To_Torque_oz=(1.0/(5310/60.0)) * 343.4;
const double c_CIM_Vel_To_Torque_nm=c_CIM_Vel_To_Torque_oz*c_OunceInchToNewton; 
const double c_CIM_Torque_to_Vel_nm=1.0 / c_CIM_Vel_To_Torque_nm;

__inline double Drive_Train_Characteristics::GetAmp_To_Torque_nm(double Amps) const 
{
	const EncoderSimulation_Props::Motor_Specs &_=m_Props.motor;
	const double c_Amp_To_Torque_nm=(1.0/(_.Stall_Current_Amp-_.Free_Current_Amp)) * _.Stall_Torque_NM;
	return max ((Amps-_.Free_Current_Amp) * c_Amp_To_Torque_nm,0.0);
}

__inline double Drive_Train_Characteristics::INV_GetVel_To_Torque_nm(double Vel_rps) const 
{
	const EncoderSimulation_Props::Motor_Specs &_=m_Props.motor;
	const double c_Vel_To_Torque_nm=(1.0/(_.FreeSpeed_RPM/60.0)) * _.Stall_Torque_NM;
	return (Vel_rps * c_Vel_To_Torque_nm);
}

__inline double Drive_Train_Characteristics::GetVel_To_Torque_nm(double motor_Vel_rps) const
{
	const EncoderSimulation_Props::Motor_Specs &_=m_Props.motor;
	const double FreeSpeed_RPS=(_.FreeSpeed_RPM/60.0);
	//ensure we don't exceed the max velocity
	const double x=min(fabs(motor_Vel_rps)/FreeSpeed_RPS,1.0);  //working with normalized positive number for inversion
	const double Torque_nm ((1.0-x) * _.Stall_Torque_NM);
	return (motor_Vel_rps>=0)? Torque_nm : -Torque_nm;
}
//gear reduction (5310/60.0) / (427.68 / 60.0) = 12.415824915824915824915824915825
//TODO compute gear reduction from max speed and pass into props here
Drive_Train_Characteristics::Drive_Train_Characteristics()
{
	EncoderSimulation_Properties default_props;
	m_Props=default_props.GetEncoderSimulationProps();
}
__inline double Drive_Train_Characteristics::GetWheelTorque(double Torque) const
{
	return Torque / m_Props.GearReduction * m_Props.DriveTrainEfficiency;
}

__inline double Drive_Train_Characteristics::INV_GetWheelTorque(double Torque) const
{
	return Torque * m_Props.GearReduction * m_Props.COF_Efficiency;
}

__inline double Drive_Train_Characteristics::GetTorqueAtWheel(double Torque) const
{
	return (GetWheelTorque(Torque) / m_Props.DriveWheelRadius);
}
__inline double Drive_Train_Characteristics::GetWheelRPS(double LinearVelocity) const
{
	return LinearVelocity / (M_PI * 2.0 * m_Props.DriveWheelRadius);			
}
__inline double Drive_Train_Characteristics::GetLinearVelocity(double wheel_RPS) const
{
	return wheel_RPS * (M_PI * 2.0 * m_Props.DriveWheelRadius);
}
__inline double Drive_Train_Characteristics::GetLinearVelocity_WheelAngular(double wheel_AngularVelocity) const
{
	return wheel_AngularVelocity * m_Props.DriveWheelRadius;
}
__inline double Drive_Train_Characteristics::GetMotorRPS(double LinearVelocity) const 
{
	return GetWheelRPS(LinearVelocity) /  m_Props.GearReduction;
}

__inline double Drive_Train_Characteristics::GetWheelRPS_Angular(double wheel_AngularVelocity) const
{
	return wheel_AngularVelocity / (M_PI * 2.0);			
}

__inline double Drive_Train_Characteristics::GetWheelAngular_RPS(double wheel_RPS) const
{
	return wheel_RPS * (M_PI * 2.0);			
}
__inline double Drive_Train_Characteristics::GetWheelAngular_LinearVelocity(double LinearVelocity) const
{
	return LinearVelocity / m_Props.DriveWheelRadius;
}
__inline double Drive_Train_Characteristics::GetMotorRPS_Angular(double wheel_AngularVelocity) const
{
	return GetWheelRPS_Angular(wheel_AngularVelocity) /  m_Props.GearReduction;
}
__inline double Drive_Train_Characteristics::INV_GetMotorRPS_Angular(double wheel_AngularVelocity) const
{
	return GetWheelRPS_Angular(wheel_AngularVelocity) *  m_Props.GearReduction;
}

__inline double Drive_Train_Characteristics::GetTorqueFromLinearVelocity(double LinearVelocity) const
{
	const double MotorTorque=GetVel_To_Torque_nm(GetMotorRPS(LinearVelocity));
	return GetTorqueAtWheel(MotorTorque);
}

__inline double Drive_Train_Characteristics::GetWheelTorqueFromVoltage(double Voltage) const
{
	const EncoderSimulation_Props::Motor_Specs &motor=m_Props.motor;
	const double Amps=fabs(Voltage*motor.Stall_Current_Amp);
	const double MotorTorque=GetAmp_To_Torque_nm(Amps);
	const double WheelTorque=GetTorqueAtWheel(MotorTorque * 2.0);
	return (Voltage>0)? WheelTorque : -WheelTorque;  //restore sign
}

__inline double Drive_Train_Characteristics::GetTorqueFromVoltage_V1(double Voltage) const
{
	const EncoderSimulation_Props::Motor_Specs &motor=m_Props.motor;
	const double Amps=fabs(Voltage*motor.Stall_Current_Amp);
	const double MotorTorque=GetAmp_To_Torque_nm(Amps);
	const double WheelTorque=INV_GetWheelTorque(MotorTorque * m_Props.NoMotors);
	return (Voltage>0)? WheelTorque : -WheelTorque;  //restore sign
}

__inline double Drive_Train_Characteristics::GetTorqueFromVoltage(double Voltage) const
{
	const EncoderSimulation_Props::Motor_Specs &motor=m_Props.motor;
	const double Amps=fabs(Voltage*motor.Stall_Current_Amp);
	const double MotorTorque=GetAmp_To_Torque_nm(Amps);
	const double WheelTorque=GetWheelTorque(MotorTorque * m_Props.NoMotors);
	return (Voltage>0)? WheelTorque : -WheelTorque;  //restore sign
}

__inline double Drive_Train_Characteristics::INV_GetTorqueFromVelocity(double wheel_AngularVelocity) const
{
	const double MotorTorque_nm=INV_GetVel_To_Torque_nm(INV_GetMotorRPS_Angular(wheel_AngularVelocity));
	return INV_GetWheelTorque(MotorTorque_nm * m_Props.NoMotors);
}

__inline double Drive_Train_Characteristics::GetTorqueFromVelocity(double wheel_AngularVelocity) const
{
	const double MotorTorque_nm=GetVel_To_Torque_nm(GetMotorRPS_Angular(wheel_AngularVelocity));
	return GetWheelTorque(MotorTorque_nm * m_Props.NoMotors);
}
__inline double Drive_Train_Characteristics::GetMaxPushingForce() const
{	
	return std::min(GetMaxTraction()*9.80665, GetMaxDriveForce()); 
}
  /***************************************************************************************************************/
 /*												Encoder_Simulator2												*/
/***************************************************************************************************************/


Encoder_Simulator2::Encoder_Simulator2(const char EntityName[]) : m_Time_s(0.0),m_EncoderScalar(1.0),m_ReverseMultiply(1.0),m_Position(0.0)
{
}

void Encoder_Simulator2::Initialize(const Ship_1D_Properties *props)
{
	const Rotary_Properties *rotary_props=dynamic_cast<const Rotary_Properties *>(props);
	if (rotary_props)
	{
		m_DriveTrain.UpdateProps(rotary_props->GetEncoderSimulationProps());
		m_EncoderScalar=rotary_props->GetRotaryProps().EncoderToRS_Ratio;
	}

	#if 0
	//m_Physics.SetMass(68);  //(about 150 pounds)
	m_Physics.SetMass(50);  //adjust this to match latency we observe
	m_Physics.SetFriction(0.8,0.08);
	#else
	m_Physics.SetMass(m_DriveTrain.GetDriveTrainProps().Wheel_Mass);
	m_Physics.SetFriction(0.8,0.2);
	m_Physics.SetRadiusOfConcentratedMass(m_DriveTrain.GetDriveTrainProps().DriveWheelRadius);
	#endif
}

void Encoder_Simulator2::UpdateEncoderVoltage(double Voltage)
{
	double Direction=Voltage<0 ? -1.0 : 1.0;
	Voltage=fabs(Voltage); //make positive
	//Apply the victor curve
	//Apply the polynomial equation to the voltage to linearize the curve
	{
		const double *c=Polynomial;
		double x2=Voltage*Voltage;
		double x3=Voltage*x2;
		double x4=x2*x2;
		Voltage = (c[4]*x4) + (c[3]*x3) + (c[2]*x2) + (c[1]*Voltage) + c[0]; 
		Voltage *= Direction;
	}
	//From this point it is a percentage (hopefully linear distribution after applying the curve) of the max force to apply... This can be
	//computed from stall torque ratings of motor with various gear reductions and so forth
	//on JVN's spread sheet torque at wheel is (WST / DWR) * 2  (for nm)  (Wheel stall torque / Drive Wheel Radius * 2 sides)
	//where stall torque is ST / GearReduction * DriveTrain efficiency
	//This is all computed in the drive train
	#if 0
	double ForceToApply=m_DriveTrain.GetWheelTorqueFromVoltage(Voltage);
	const double ForceAbsorbed=m_DriveTrain.GetWheelTorqueFromVelocity(m_Physics.GetVelocity());
	ForceToApply-=ForceAbsorbed;
	m_Physics.ApplyFractionalForce(ForceToApply,m_Time_s);
	#else
	double TorqueToApply=m_DriveTrain.GetTorqueFromVoltage_V1(Voltage);
	const double TorqueAbsorbed=m_DriveTrain.INV_GetTorqueFromVelocity(m_Physics.GetVelocity());
	TorqueToApply-=TorqueAbsorbed;
	m_Physics.ApplyFractionalTorque(TorqueToApply,m_Time_s,m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
	#endif
}

double Encoder_Simulator2::GetEncoderVelocity() const
{
	#if 0
	static size_t i=0;
	double Velocity=m_Physics.GetVelocity();
	if (((i++ % 50)==0)&&(Velocity!=0.0))
		printf("Velocty=%f\n",Velocity *  m_DriveTrain.GetDriveTrainProps().DriveWheelRadius );
	return 0.0;
	#else
	//return m_Physics.GetVelocity();
	return m_Physics.GetVelocity() *  m_DriveTrain.GetDriveTrainProps().DriveWheelRadius * m_ReverseMultiply;
	#endif
}

double Encoder_Simulator2::GetDistance() const
{
	return m_Position;
}

void Encoder_Simulator2::SetReverseDirection(bool reverseDirection)
{
	m_ReverseMultiply= reverseDirection? -1.0 : 1.0;
}

void Encoder_Simulator2::TimeChange()
{
	//m_GetEncoderFirstCall=true;
	const double Ground=0.0;  //ground in radians
	double FrictionForce=m_Physics.GetFrictionalForce(m_Time_s,Ground);
	m_Physics.ApplyFractionalForce(FrictionForce,m_Time_s);  //apply the friction
	double PositionDisplacement;
	m_Physics.TimeChangeUpdate(m_Time_s,PositionDisplacement);
	m_Position+= PositionDisplacement * m_EncoderScalar * m_ReverseMultiply;
}

void Encoder_Simulator2::ResetPos()
{
	m_Physics.ResetVectors();
	m_Position=0;
}
  /***************************************************************************************************************/
 /*												Encoder_Simulator3												*/
/***************************************************************************************************************/
PhysicsEntity_1D Encoder_Simulator3::s_PayloadPhysics_Left;
PhysicsEntity_1D Encoder_Simulator3::s_PayloadPhysics_Right;
double Encoder_Simulator3::s_PreviousPayloadVelocity_Left=0.0;
double Encoder_Simulator3::s_PreviousPayloadVelocity_Right=0.0;

Encoder_Simulator3::Encoder_Simulator3(const char EntityName[]) :
	Encoder_Simulator2(EntityName),m_EncoderKind(eIgnorePayload),m_PreviousWheelVelocity(0.0)
{
}

void Encoder_Simulator3::SetEncoderKind(EncoderKind kind) 
{
	//Comment this out to do bench testing
	m_EncoderKind=kind;
}

void Encoder_Simulator3::Initialize(const Ship_1D_Properties *props)
{
	__super::Initialize(props);
	m_Physics.SetAngularInertiaCoefficient(0.5);  //Going for solid cylinder
	//now to setup the payload physics   Note: each side pulls half the weight
	if (m_EncoderKind==eRW_Left)
		s_PayloadPhysics_Left.SetMass(m_DriveTrain.GetDriveTrainProps().PayloadMass/2.0);
	else if (m_EncoderKind==eRW_Right)
		s_PayloadPhysics_Right.SetMass(m_DriveTrain.GetDriveTrainProps().PayloadMass/2.0);
	//TODO see if this is needed
	//m_PayloadPhysics.SetFriction(0.8,0.2);
}

void Encoder_Simulator3::UpdateEncoderVoltage(double Voltage)
{
	m_Voltage=Voltage;
	//depreciated
	#if 0
	const double Direction=Voltage<0 ? -1.0 : 1.0;
	Voltage=fabs(Voltage); //make positive
	//Apply the victor curve
	//Apply the polynomial equation to the voltage to linearize the curve
	{
		const double *c=Polynomial;
		double x2=Voltage*Voltage;
		double x3=Voltage*x2;
		double x4=x2*x2;
		Voltage = (c[4]*x4) + (c[3]*x3) + (c[2]*x2) + (c[1]*Voltage) + c[0]; 
		Voltage *= Direction;
	}
	#endif
	//This is line is somewhat subtle... basically if the voltage is in the same direction as the velocity we use the linear distribution of the curve, when they are in
	//opposite directions zero gives the stall torque where we need max current to switch directions (same amount as if there is no motion)
	const double VelocityToUse=m_Physics.GetVelocity()*Voltage > 0 ? m_Physics.GetVelocity() : 0.0;
	double TorqueToApply=m_DriveTrain.GetTorqueFromVelocity(VelocityToUse);
	//Avoid ridiculous division and zero it out... besides a motor can't really turn the wheel in the dead zone range, but I'm not going to factor that in yet
	if  ((TorqueToApply!=0.0)&&(fabs(TorqueToApply)<m_Physics.GetMass()*2))
		TorqueToApply=0.0;
	//Note: Even though TorqueToApply has direction, if it gets saturated to 0 it loses it... ultimately the voltage parameter is sacred to the correct direction
	//in all cases so we'll convert TorqueToApply to magnitude
	m_Physics.ApplyFractionalTorque(fabs(TorqueToApply) * Voltage,m_Time_s, m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);

	//Apply the appropriate change in linear velocity to the payload.  Note:  This is not necessarily torque because of the TorqueAccelerationDampener... the torque
	//acts as voltage acts to a circuit... it gives the potential but the current doesn't reflect this right away... its the current, or in our case the velocity
	//change of he wheel itself that determines how much force to apply.
	const double Wheel_Acceleration=(m_Physics.GetVelocity()-m_PreviousWheelVelocity)/m_Time_s;
	//Now to reconstruct the torque Ia, where I = cm R^2 /torquedampner
	const double Wheel_Torque=Wheel_Acceleration*m_Physics.GetMomentofInertia(m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
	//Grrr I have to blend this... mostly because of the feedback from the payload... I'll see if there is some way to avoid needing to do this
	const double smoothing_value=0.75;
	const double BlendTorqueToApply=(Wheel_Torque*smoothing_value)+(TorqueToApply*(1.0-smoothing_value));
	//TODO check for case when current drive force is greater than the traction
	//Compute the pushing force of the mass and apply it just the same
	if (m_EncoderKind==eRW_Left)
	{
		//Testing
		#if 0
		SmartDashboard::PutNumber("Wheel_Torque_Left",Wheel_Torque);
		SmartDashboard::PutNumber("Motor_Torque_Left",TorqueToApply);
		#endif
		s_PayloadPhysics_Left.ApplyFractionalForce(fabs(m_DriveTrain.GetCurrentDriveForce(BlendTorqueToApply))*Voltage,m_Time_s);
	}
	else if (m_EncoderKind==eRW_Right)
		s_PayloadPhysics_Right.ApplyFractionalForce(fabs(m_DriveTrain.GetCurrentDriveForce(BlendTorqueToApply))*Voltage,m_Time_s);
}

double Encoder_Simulator3::GetEncoderVelocity() const
{
	//we return linear velocity (which is native to the ship)
	return  m_DriveTrain.GetLinearVelocity(m_DriveTrain.GetWheelRPS_Angular(m_Physics.GetVelocity())) * m_ReverseMultiply;
}

#define __USE_PAYLOAD_MASS__
//local function to avoid redundant code
static void TimeChange_UpdatePhysics(double Voltage,
									 Drive_Train_Characteristics dtc,PhysicsEntity_1D &PayloadPhysics,PhysicsEntity_1D &WheelPhysics,double Time_s,bool UpdatePayload)
{
	const double PayloadVelocity=PayloadPhysics.GetVelocity();
	const double WheelVelocity=WheelPhysics.GetVelocity();

	double PositionDisplacement;
	//TODO add speed loss force
	if (UpdatePayload)
	{
		//apply a constant speed loss using new velocity - old velocity
		if (Voltage==0.0)
		{
			const double MaxStop=fabs(PayloadVelocity/Time_s);
			const double SpeedLoss=min(5.0,MaxStop);
			const double acceleration=PayloadVelocity>0.0?-SpeedLoss:SpeedLoss;
			//now to factor in the mass
			const double SpeedLossForce = PayloadPhysics.GetMass() * acceleration;
			PayloadPhysics.ApplyFractionalTorque(SpeedLossForce,Time_s);
		}
		PayloadPhysics.TimeChangeUpdate(Time_s,PositionDisplacement);
	}

	#ifdef __USE_PAYLOAD_MASS__
	//Now to add force normal against the wheel this is the difference between the payload and the wheel velocity
	//When the mass is lagging behind it add adverse force against the motor... and if the mass is ahead it will
	//relieve the motor and make it coast in the same direction
	//const double acceleration = dtc.GetWheelAngular_RPS(dtc.GetWheelRPS(PayloadVelocity))-WheelVelocity;
	const double acceleration = dtc.GetWheelAngular_LinearVelocity(PayloadVelocity)-WheelVelocity;
	if (PayloadVelocity!=0.0)
	{
		#if 0
		if (UpdatePayload)
			SmartDashboard::PutNumber("Test",dtc.GetWheelAngular_LinearVelocity(PayloadVelocity));
		if (fabs(acceleration)>50)
			int x=8;
		#endif
		//make sure wheel and payload are going in the same direction... 
		if (PayloadVelocity*WheelVelocity >= 0.0)
		{
			//now to factor in the mass
			const double PayloadForce = WheelPhysics.GetMomentofInertia(dtc.GetDriveTrainProps().TorqueAccelerationDampener) * acceleration;
			WheelPhysics.ApplyFractionalTorque(PayloadForce,Time_s);
		}
		//else
		//	printf("skid %.2f\n",acceleration);
		//if not... we have skidding (unless its some error of applying the force normal) for now let it continue its discourse by simply not applying the payload force
		//this will make it easy to observe cases when skids can happen, but eventually we should compute the kinetic friction to apply
	}
	else
		WheelPhysics.SetVelocity(0.0);
	#endif
}

void Encoder_Simulator3::TimeChange()
{
	//For best results if we locked to the payload we needn't apply a speed loss constant here
	#ifndef __USE_PAYLOAD_MASS__
	//first apply a constant speed loss using new velocity - old velocity
	//if (fabs(m_Voltage)<0.65)
	{
		if ((fabs(m_Physics.GetVelocity())>0.01))
		{
			const double acceleration = m_DriveTrain.GetDriveTrainProps().SpeedLossConstant*m_Physics.GetVelocity()-m_Physics.GetVelocity();
			//now to factor in the mass
			const double SpeedLossForce = m_Physics.GetMass() * acceleration;
			const double VoltageMagnitue=fabs(m_Voltage);
			m_Physics.ApplyFractionalTorque(SpeedLossForce,m_Time_s,VoltageMagnitue>0.5?(1.0-VoltageMagnitue)*0.5:0.25);
		}
		else
			m_Physics.SetVelocity(0.0);
	}
	#endif

	double PositionDisplacement;
	m_Physics.TimeChangeUpdate(m_Time_s,PositionDisplacement);
	m_Position+= PositionDisplacement * m_EncoderScalar * m_ReverseMultiply;

	//cache velocities
	m_PreviousWheelVelocity=m_Physics.GetVelocity();
	if ((m_EncoderKind==eRW_Left)||(m_EncoderKind==eReadOnlyLeft))
	{
		TimeChange_UpdatePhysics(m_Voltage,m_DriveTrain,s_PayloadPhysics_Left,m_Physics,m_Time_s,m_EncoderKind==eRW_Left);
		if (m_EncoderKind==eRW_Left)
		{
			s_PreviousPayloadVelocity_Left=s_PayloadPhysics_Left.GetVelocity();
			SmartDashboard::PutNumber("PayloadLeft",Meters2Feet(s_PreviousPayloadVelocity_Left));
			SmartDashboard::PutNumber("WheelLeft",Meters2Feet(GetEncoderVelocity()));
		}
	}
	else if ((m_EncoderKind==eRW_Right)||(m_EncoderKind==eReadOnlyRight))
	{
		TimeChange_UpdatePhysics(m_Voltage,m_DriveTrain,s_PayloadPhysics_Right,m_Physics,m_Time_s,m_EncoderKind==eRW_Right);
		if (m_EncoderKind==eReadOnlyRight)
		{
			s_PreviousPayloadVelocity_Right=s_PayloadPhysics_Right.GetVelocity();
			SmartDashboard::PutNumber("PayloadRight",Meters2Feet(s_PreviousPayloadVelocity_Right));
			SmartDashboard::PutNumber("WheelRight",Meters2Feet(GetEncoderVelocity()));
		}
	}
}

void Encoder_Simulator3::ResetPos()
{
	__super::ResetPos();
	if (m_EncoderKind==eRW_Left)
		s_PayloadPhysics_Left.ResetVectors();
	if (m_EncoderKind==eRW_Right)
		s_PayloadPhysics_Right.ResetVectors();
}

  /***************************************************************************************************************/
 /*													Encoder_Tester												*/
/***************************************************************************************************************/

Encoder_Tester::Encoder_Tester() : m_LeftEncoder("LeftEncoder"),m_RightEncoder("RightEncoder")
{
	m_LeftEncoder.Initialize(NULL);
	m_RightEncoder.Initialize(NULL);
}

void Encoder_Tester::Initialize(const Ship_1D_Properties *props)
{
	m_LeftEncoder.Initialize(props);
	m_RightEncoder.Initialize(props);
	//Cool way to add friction uneven to simulate 
	//m_RightEncoder.SetFriction(0.8,0.12);
}

void Encoder_Tester::GetLeftRightVelocity(double &LeftVelocity,double &RightVelocity)
{
	LeftVelocity=m_LeftEncoder.GetEncoderVelocity();
	RightVelocity=m_RightEncoder.GetEncoderVelocity();
}

void Encoder_Tester::UpdateLeftRightVoltage(double LeftVoltage,double RightVoltage)
{
	//LeftVoltage=GetTweakedVoltage(LeftVoltage);
	//RightVoltage=GetTweakedVoltage(RightVoltage);
	m_LeftEncoder.UpdateEncoderVoltage(LeftVoltage);
	m_RightEncoder.UpdateEncoderVoltage(RightVoltage);
}

void Encoder_Tester::SetTimeDelta(double dTime_s)
{
	m_LeftEncoder.SetTimeDelta(dTime_s);
	m_RightEncoder.SetTimeDelta(dTime_s);
}

void Encoder_Tester::TimeChange()
{
	m_LeftEncoder.TimeChange();
	m_RightEncoder.TimeChange();
}
#endif