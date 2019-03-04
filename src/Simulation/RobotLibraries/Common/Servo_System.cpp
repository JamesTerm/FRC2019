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
#include "Robot_Control_Interface.h"
#include "Servo_System.h"

using namespace Framework::Base;
using namespace std;


  /***********************************************************************************************************************************/
 /*														Servo_Position_Control														*/
/***********************************************************************************************************************************/

Servo_Position_Control::Servo_Position_Control(const char EntityName[],Servo_Control_Interface *robot_control,size_t InstanceIndex) : Ship_1D(EntityName),
	m_RobotControl(robot_control),m_InstanceIndex(InstanceIndex),
	m_LastPosition(0.0),m_MatchVelocity(0.0),
	m_LastTime(0.0),m_PreviousVelocity(0.0)
{
}

void Servo_Position_Control::Initialize(Base::EventMap& em,const Entity1D_Properties *props)
{
	__super::Initialize(em,props);
	const Servo_Properties *Props=dynamic_cast<const Servo_Properties *>(props);
	assert(Props);
	//This will copy all the props
	m_Servo_Props=Props->GetServoProps();
}

void Servo_Position_Control::TimeChange(double dTime_s)
{
	//Note: the order has to be in this order where it grabs the potentiometer position first and then performs the time change and finally updates the
	//new arm velocity.  Doing it this way avoids oscillating if the potentiometer and gear have been calibrated
	if (!m_LastTime) 
	{
		m_LastTime=dTime_s;
		#ifdef Robot_TesterCode
		assert(dTime_s!=0.0);
		#endif
	}

	//const double NewPosition=m_RobotControl->GetRotaryCurrentPorV(m_InstanceIndex);
	//const double Displacement=NewPosition-m_LastPosition;
	//const double PotentiometerVelocity=Displacement/m_LastTime;

	//m_LastPosition=NewPosition;
	//m_LastTime=dTime_s;

	__super::TimeChange(dTime_s);


	//#ifdef __DebugLUA__
	//if ((m_Servo_Props.Console_Dump))
	//{
	//	double PosY=m_LastPosition;
	//	//double PosY=RAD_2_DEG(m_LastPosition);
	//	printf("y=%.2f p=%f\n",PosY,CurrentVelocity);
	//}
	//#endif

	//Note: the scaling of angle occurs in robot control
	m_RobotControl->SetServoAngle(m_InstanceIndex,GetPos_m());
}


void Servo_Position_Control::ResetPos()
{
	__super::ResetPos();  //Let the super do it stuff first
	if (!GetBypassPos_Update())
	{
		m_RobotControl->Reset_Servo(m_InstanceIndex);
		double NewPosition=m_RobotControl->GetServoAngle(m_InstanceIndex);
		Stop();
		SetPos_m(NewPosition);
		m_LastPosition=NewPosition;
	}
}

  /***********************************************************************************************************************************/
 /*														Servo_Properties															*/
/***********************************************************************************************************************************/

void Servo_Properties::Init()
{
	Servo_Props props;
	memset(&props,0,sizeof(Servo_Props));

	props.ServoScalar=1.0;
	props.ServoOffset=0.0;
	//Late assign this to override the initial default
	props.PrecisionTolerance=DEG_2_RAD(0.05); 
	props.Feedback_DiplayRow=(size_t)-1;  //Only assigned to a row during calibration of feedback sensor
	m_ServoProps=props;
}

void Servo_Properties::LoadFromScript(Scripting::Script& script, bool NoDefaults)
{
	const char* err=NULL;

	//I shouldn't need this nested field redundancy... just need to be sure all client cases like this
	//err = script.GetFieldTable("rotary_settings");
	//if (!err) 

	{

		double fValue;
		SCRIPT_INIT_DOUBLE(m_ServoProps.ServoScalar,		 "servo_ratio");
		SCRIPT_INIT_DOUBLE(m_ServoProps.ServoOffset,		 "servo_offset");
		SCRIPT_INIT_DOUBLE(m_ServoProps.PrecisionTolerance,"tolerance");

		err=script.GetField("ds_display_row", NULL, NULL, &fValue);
		if (!err)
			m_ServoProps.Feedback_DiplayRow=(size_t)fValue;

		string sTest;
		//err = script.GetField("show_pid_dump",&sTest,NULL,NULL);
		//if (!err)
		//{
		//	if ((sTest.c_str()[0]=='y')||(sTest.c_str()[0]=='Y')||(sTest.c_str()[0]=='1'))
		//		m_ServoProps.PID_Console_Dump=true;
		//}
	}
	__super::LoadFromScript(script,NoDefaults);
}
