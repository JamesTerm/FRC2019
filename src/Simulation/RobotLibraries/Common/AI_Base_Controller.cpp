#include "../Base/Base_Includes.h"
#include <math.h>
#include <algorithm>
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
#include "Ship.h"
#include "AI_Base_Controller.h"
#include "../Base/Joystick.h"
#include "../Base/JoystickBinder.h"
#include "UI_Controller.h"

using namespace Framework::Base;


  /***********************************************************************************************************************************/
 /*														LUA_Controls_Properties														*/
/***********************************************************************************************************************************/

LUA_Controls_Properties::LUA_Controls_Properties(LUA_Controls_Properties_Interface *parent) : m_pParent(parent)
{
}

LUA_Controls_Properties &LUA_Controls_Properties::operator= (const LUA_Controls_Properties &CopyFrom)
{
	m_Controls=CopyFrom.m_Controls;
	//Note: this one probably requires a static interface
	m_pParent=CopyFrom.m_pParent;
	return *this;
}

const char *LUA_Controls_Properties::ExtractControllerElementProperties(Controller_Element_Properties &Element,const char *Eventname,Scripting::Script& script)
{
	const char *err=NULL;
	err = script.GetFieldTable(Eventname);
	if (!err)
	{
		Element.Event=Eventname;
		std::string sType;
		err = script.GetField("type",&sType,NULL,NULL);
		ASSERT_MSG(!err, err);
		
		if (strcmp(sType.c_str(),"joystick_analog")==0)
		{
			Element.Type=Controller_Element_Properties::eJoystickAnalog;
			JoyAxis_enum JoyAxis;
			double dJoyAxis;
			err = script.GetField("key", NULL, NULL,&dJoyAxis);
			ASSERT_MSG(!err, err);
			//cast to int first, and then to the enumeration
			JoyAxis=(JoyAxis_enum)((int)dJoyAxis);
			bool IsFlipped;
			err = script.GetField("is_flipped", NULL, &IsFlipped,NULL);
			ASSERT_MSG(!err, err);
			double Multiplier;
			err = script.GetField("multiplier", NULL, NULL,&Multiplier);
			ASSERT_MSG(!err, err);
			double FilterRange;
			err = script.GetField("filter", NULL, NULL,&FilterRange);
			ASSERT_MSG(!err, err);
			double CurveIntensity;
			err = script.GetField("curve_intensity", NULL, NULL, &CurveIntensity);
			ASSERT_MSG(!err, err);

			Controller_Element_Properties::ElementTypeSpecific::AnalogSpecifics_rw &set=Element.Specifics.Analog;
			set.JoyAxis=JoyAxis;
			set.IsFlipped=IsFlipped;
			set.Multiplier=Multiplier;
			set.FilterRange=FilterRange;
			set.CurveIntensity=CurveIntensity;
			//joy.AddJoy_Analog_Default(JoyAxis,Eventname,IsFlipped,Multiplier,FilterRange,IsSquared,ProductName.c_str());
		}
		else if (strcmp(sType.c_str(),"joystick_dual_analog")==0)
		{
			Element.Type=Controller_Element_Properties::eJoystickSplitAxis;
			JoyAxis_enum JoyAxis_1,JoyAxis_2;
			double dJoyAxis;
			err = script.GetField("key_1", NULL, NULL,&dJoyAxis);
			ASSERT_MSG(!err, err);
			//cast to int first, and then to the enumeration
			JoyAxis_1=(JoyAxis_enum)((int)dJoyAxis);

			err = script.GetField("key_2", NULL, NULL,&dJoyAxis);
			ASSERT_MSG(!err, err);
			//cast to int first, and then to the enumeration
			JoyAxis_2=(JoyAxis_enum)((int)dJoyAxis);

			bool CenterPointIdle;
			err = script.GetField("center_point_idle", NULL, &CenterPointIdle,NULL);
			if (err)
				CenterPointIdle=true;  //default
			bool IsFlipped;
			err = script.GetField("is_flipped", NULL, &IsFlipped,NULL);
			ASSERT_MSG(!err, err);
			double Multiplier;
			err = script.GetField("multiplier", NULL, NULL,&Multiplier);
			ASSERT_MSG(!err, err);
			double FilterRange;
			err = script.GetField("filter", NULL, NULL,&FilterRange);
			ASSERT_MSG(!err, err);
			double CurveIntensity;
			err = script.GetField("curve_intensity", NULL, NULL, &CurveIntensity);
			ASSERT_MSG(!err, err);

			Controller_Element_Properties::ElementTypeSpecific::SplitAxisSpecifics_rw &set=Element.Specifics.DualAnalog;
			set.JoyAxis_1=JoyAxis_1,set.JoyAxis_2=JoyAxis_2;
			set.CenterPointIdle=CenterPointIdle;
			set.IsFlipped=IsFlipped;
			set.Multiplier=Multiplier;
			set.FilterRange=FilterRange;
			set.CurveIntensity=CurveIntensity;
		}
		else if (strcmp(sType.c_str(),"joystick_culver")==0)
		{
			Element.Type=Controller_Element_Properties::eJoystickCulver;
			JoyAxis_enum JoyAxis_X,JoyAxis_Y;
			double dJoyAxis;
			err = script.GetField("key_x", NULL, NULL,&dJoyAxis);
			ASSERT_MSG(!err, err);
			//cast to int first, and then to the enumeration
			JoyAxis_X=(JoyAxis_enum)((int)dJoyAxis);

			err = script.GetField("key_y", NULL, NULL,&dJoyAxis);
			ASSERT_MSG(!err, err);
			//cast to int first, and then to the enumeration
			JoyAxis_Y=(JoyAxis_enum)((int)dJoyAxis);

			bool IsFlipped;
			err = script.GetField("is_flipped", NULL, &IsFlipped,NULL);
			ASSERT_MSG(!err, err);
			double Magnitude_Scalar_Arc;
			err = script.GetField("magnitude_scalar_arc", NULL, NULL,&Magnitude_Scalar_Arc);
			if (err)
				Magnitude_Scalar_Arc=1.0/PI_2;  //this is a great default for game controllers that have 1.0 intensity at the corners
			double Magnitude_Scalar_Base;
			err = script.GetField("magnitude_scalar_base", NULL, NULL,&Magnitude_Scalar_Base);
			if (err)
				Magnitude_Scalar_Base=1.0/PI_2;  //this is a great default for all (may need to be slightly tweaked to get perfect on some controllers)

			double Multiplier;
			err = script.GetField("multiplier", NULL, NULL,&Multiplier);
			ASSERT_MSG(!err, err);
			double FilterRange;
			err = script.GetField("filter", NULL, NULL,&FilterRange);
			ASSERT_MSG(!err, err);
			double CurveIntensity;
			err = script.GetField("curve_intensity", NULL, NULL, &CurveIntensity);
			ASSERT_MSG(!err, err);

			Controller_Element_Properties::ElementTypeSpecific::CulverSpecifics_rw &set=Element.Specifics.Culver;
			set.JoyAxis_X=JoyAxis_X,set.JoyAxis_Y=JoyAxis_Y;
			set.MagnitudeScalarArc=Magnitude_Scalar_Arc;
			set.MagnitudeScalarBase=Magnitude_Scalar_Base;
			set.IsFlipped=IsFlipped;
			set.Multiplier=Multiplier;
			set.FilterRange=FilterRange;
			set.CurveIntensity=CurveIntensity;
			//joy.AddJoy_Culver_Default(JoyAxis_X,JoyAxis_Y,Magnitude_Scalar,Eventname,IsFlipped,Multiplier,FilterRange,CurveIntensity,ProductName.c_str());
		}
		else if (strcmp(sType.c_str(),"joystick_button")==0)
		{
			Element.Type=Controller_Element_Properties::eJoystickButton;
			size_t WhichButton,WhichKey;
			double dWhichButton;
			err = script.GetField("key", NULL, NULL,&dWhichButton);
			ASSERT_MSG(!err, err);

			std::string stringWhichButton;
			err = script.GetField("keyboard",&stringWhichButton, NULL, NULL);
			if (!err)
				WhichKey=stringWhichButton.c_str()[0];
			else
				WhichKey=(size_t)-1;

			//cast to int first, and then to the enumeration; The -1 allows for cardinal types (good since we can use numbers written on button)
			WhichButton=(JoyAxis_enum)((int)dWhichButton-1);
			bool useOnOff;
			err = script.GetField("on_off", NULL, &useOnOff,NULL);
			ASSERT_MSG(!err, err);
			bool dbl_click=false;
			err = script.GetField("dbl", NULL, &dbl_click,NULL); //This one can be blank
			err=NULL;  //don't return an error (assert for rest)

			Controller_Element_Properties::ElementTypeSpecific::ButtonSpecifics_rw &set=Element.Specifics.Button;
			set.WhichButton=WhichButton;
			set.WhichKey=WhichKey;
			set.useOnOff=useOnOff;
			set.dbl_click=dbl_click;
			//joy.AddJoy_Button_Default( WhichButton,Eventname,useOnOff,dbl_click,ProductName.c_str());
		}
		else if (strcmp(sType.c_str(),"keyboard")==0)
		{
			Element.Type=Controller_Element_Properties::eKeyboard;
			size_t WhichKey;
			std::string stringWhichButton;
			err = script.GetField("key",&stringWhichButton, NULL, NULL);
			ASSERT_MSG(!err, err);
			//cast to int first, and then to the enumeration; The -1 allows for cardinal types (good since we can use numbers written on button)
			WhichKey=stringWhichButton.c_str()[0];
			bool useOnOff;
			err = script.GetField("on_off", NULL, &useOnOff,NULL);
			ASSERT_MSG(!err, err);
			bool dbl_click=false;
			err = script.GetField("dbl", NULL, &dbl_click,NULL); //This one can be blank
			err=NULL;  //don't return an error (assert for rest)

			Controller_Element_Properties::ElementTypeSpecific::KeyboardSpecifics_rw &set=Element.Specifics.Keyboard;
			set.WhichKey=WhichKey;
			set.useOnOff=useOnOff;
			set.dbl_click=dbl_click;
		}
		else assert(false);
		script.Pop();
	}
	return err;
}

void LUA_Controls_Properties::LoadFromScript(Scripting::Script& script)
{
	const char* err=NULL;
	char Buffer[4];

	m_DriverStation_SlotList.clear();
	err=script.GetFieldTable("slotlist");
	if (!err)
	{
		//Note i is cardinal (more readable in LUA)
		size_t i=1;
		std::string Slot;
		std::string SlotValue;
		while ( Slot="slot_",Slot+=itoa((int)i++,Buffer,10) ,	(err = script.GetField(Slot.c_str(),&SlotValue,NULL,NULL))==NULL)
			m_DriverStation_SlotList.push_back(SlotValue);
		assert(m_DriverStation_SlotList.size()<=4); //only up to 4 slots supported
		script.Pop();
	}

	//ensure the list is clean (incase it gets called again)
	m_Controls.clear();

	//Note i is cardinal (more readable in LUA)
	size_t i=1,j=0;
	std::string Controls;
	const char * Events;
	while ( Controls="Joystick_",Controls+=itoa((int)i++,Buffer,10) ,	(err = script.GetFieldTable(Controls.c_str()))==NULL)
	{
		size_t AxisCount=6;
		Control_Props control;
		#ifndef Robot_TesterCode
		//We can use the product name if we provided the slot list... testing only if we used a slot list
		if (m_DriverStation_SlotList.size()>0)
		{
			//now see if the controller we are on matches the slot list entry
			std::string TestController;
			err=script.GetField("control", &TestController, NULL, NULL);
			bool MatchFound=false;
			for (j=0;j<m_DriverStation_SlotList.size();j++)
			{
				if (strcmp(TestController.c_str(),m_DriverStation_SlotList[j].c_str())==0)
				{
					//I'll keep this disabled code in just for reference, basically if I forget in main to add this line:
					//m_Joystick.SetSlotList(m_RobotProps.Get_RobotControls().GetDriverStation_SlotList());
					//Then the disabled code would work.  If a remembered to make this call then the enabled line
					//below will work, and should work by itself since we would be working with product name
					//I like this filtering for the correct case so I'll leave it in as it makes a clean map of just the stuff we
					//want.
					#if 0
					//rename the controls index to the slot index found
					Controls="Joystick_";
					Controls+=itoa(j+1,Buffer,10);  //j needs to be cardinal
					control.Controller=Controls.c_str();
					#else
					err=script.GetField("control", &control.Controller, NULL, NULL);
					#endif
					MatchFound=true;
					break;
				}
			}
			if (MatchFound==false)  //no match found
			{
				script.Pop();
				continue;
			}
		}
		else  //legacy matching
		{
			//Wind River uses generic name, and AI tester uses product name
			control.Controller=Controls.c_str();
		}
		
		#else
		err=script.GetField("control", &control.Controller, NULL, NULL);
		#endif

		double fTest;
		err=script.GetField("axis_count", NULL, NULL, &fTest);
		if (!err)
			AxisCount=(size_t)fTest;
		//ensure the controller is lower case
		std::transform(control.Controller.begin(),control.Controller.end(),control.Controller.begin(),tolower);
		j=0;
		while ( Events=m_pParent->LUA_Controls_GetEvents(j++) , Events)
		{
			Controller_Element_Properties element;
			err=ExtractControllerElementProperties(element,Events,script);
			if (!err)
				control.EventList.push_back(element);
		}
		control.AxisCount=AxisCount;
		m_Controls.push_back(control);
		script.Pop();
	}
}

void LUA_Controls_Properties::BindAdditionalUIControls(bool Bind,void *joy,void *key) const
{
	typedef UI::JoyStick_Binder JoyStick_Binder;
	typedef UI::KeyboardMouse_CB Keyboard_Binder;
	JoyStick_Binder *p_joy=(JoyStick_Binder *)joy;
	Keyboard_Binder *p_key=(Keyboard_Binder *)key;
	const Controls_List &controls=Get_Controls();
	for (size_t i=0;i<controls.size();i++)
	{
		const Control_Props &control=controls[i];

		for (size_t j=0;j<control.EventList.size();j++)
		{
			const Controller_Element_Properties &element=control.EventList[j];
			switch (element.Type)
			{
			case Controller_Element_Properties::eJoystickAnalog:
				if (Bind)
				{
					const Controller_Element_Properties::ElementTypeSpecific::AnalogSpecifics_rw &analog=element.Specifics.Analog;
					//Note the cast... these are not going to change, but there is dup code to on axis enum to avoid dependency issues
					p_joy->AddJoy_Analog_Default((JoyStick_Binder::JoyAxis_enum)analog.JoyAxis,element.Event.c_str(),analog.IsFlipped,analog.Multiplier,
						analog.FilterRange,analog.CurveIntensity,control.Controller.c_str());
				}
				else
					p_joy->RemoveJoy_Analog_Binding(element.Event.c_str(),control.Controller.c_str());
				break;
			case Controller_Element_Properties::eJoystickCulver:
				if (Bind)
				{
					const Controller_Element_Properties::ElementTypeSpecific::CulverSpecifics_rw &analog=element.Specifics.Culver;
					//Note the cast... these are not going to change, but there is dup code to on axis enum to avoid dependency issues
					p_joy->AddJoy_Culver_Default((JoyStick_Binder::JoyAxis_enum)analog.JoyAxis_X,(JoyStick_Binder::JoyAxis_enum)analog.JoyAxis_Y,analog.MagnitudeScalarArc,
						analog.MagnitudeScalarBase,element.Event.c_str(),analog.IsFlipped,analog.Multiplier,analog.FilterRange,analog.CurveIntensity,control.Controller.c_str());
				}
				else
					p_joy->RemoveJoy_Analog_Binding(element.Event.c_str(),control.Controller.c_str());
				break;
			case Controller_Element_Properties::eJoystickSplitAxis:
				if (Bind)
				{
					const Controller_Element_Properties::ElementTypeSpecific::SplitAxisSpecifics_rw &analog=element.Specifics.DualAnalog;
					//Note the cast... these are not going to change, but there is dup code to on axis enum to avoid dependency issues
					p_joy->AddJoy_SplitAxis_Default((JoyStick_Binder::JoyAxis_enum)analog.JoyAxis_1,(JoyStick_Binder::JoyAxis_enum)analog.JoyAxis_2,analog.CenterPointIdle,element.Event.c_str(),analog.IsFlipped,analog.Multiplier,analog.FilterRange,analog.CurveIntensity,control.Controller.c_str());
				}
				else
					p_joy->RemoveJoy_Analog_Binding(element.Event.c_str(),control.Controller.c_str());
				break;
			case Controller_Element_Properties::eJoystickButton:
				if (Bind)
				{
					const Controller_Element_Properties::ElementTypeSpecific::ButtonSpecifics_rw &button=element.Specifics.Button;
					p_joy->AddJoy_Button_Default(button.WhichButton,element.Event.c_str(),button.useOnOff,button.dbl_click,control.Controller.c_str());
					if ((p_key) && (button.WhichKey!=(size_t)-1))
					{
						if (Bind)
							p_key->AddKeyBindingR(button.useOnOff,element.Event.c_str(),(int)button.WhichKey);
						else
							p_key->RemoveKeyBinding((int)button.WhichKey,element.Event.c_str(),button.useOnOff);
					}
				}
				else
					p_joy->RemoveJoy_Button_Binding(element.Event.c_str(),control.Controller.c_str());
				break;
			case Controller_Element_Properties::eKeyboard:
				if (p_key)
				{
					const Controller_Element_Properties::ElementTypeSpecific::KeyboardSpecifics_rw &button=element.Specifics.Keyboard;
					if (Bind)
						p_key->AddKeyBindingR(button.useOnOff,element.Event.c_str(), (int)button.WhichKey);
					else
						p_key->RemoveKeyBinding((int)button.WhichKey,element.Event.c_str(),button.useOnOff);
				}
				break;
			}
		}
	}
}

  /***********************************************************************************************************************************/
 /*															Tank_Steering															*/
/***********************************************************************************************************************************/

Tank_Steering::Tank_Steering(Ship_2D *parent) : m_pParent(parent),m_LeftVelocity(0.0),m_RightVelocity(0.0),m_LeftXAxis(0.0),m_RightXAxis(0.0),
	m_StraightDeadZone_Tolerance(0.05),m_90DegreeTurnValve(false),m_AreControlsDisabled(false)
{
}

  //We may want to tweak these in LUA
const double Tank_Steering_XAxisSignal_Threshold=0.9;
const double Tank_Steering_XAxisSignal_MinThreshold=0.2;

void Tank_Steering::UpdateController(double &AuxVelocity,Vec2D &LinearAcceleration,double &AngularAcceleration,const Ship_2D &ship,bool &LockShipHeadingToOrientation,double dTime_s)
{
	if (ship.GetAlterTrajectory())
		AuxVelocity=((m_LeftVelocity + m_RightVelocity) * 0.5) * ship.GetEngaged_Max_Speed();
	else
	{
		//Haha this is absolutely silly driving tank steering in slide mode, but it works
		LinearAcceleration[1]+=((m_LeftVelocity + m_RightVelocity) * 0.5) * ship.GetAccelSpeed();
	}
	const double difference=(m_LeftVelocity + -m_RightVelocity);
	const double omega = (fabs(difference)>m_StraightDeadZone_Tolerance)? difference * 0.5 : 0;
	//Note: there was an angular scalar here, but since we have increased the max torque high we shouldn't scale this
	//  [7/28/2013 Terminator]
	AngularAcceleration=omega*ship.GetHeadingSpeed();
	if (!IsZero(omega))
		LockShipHeadingToOrientation=true;

	//DOUT4("%f %f %f",m_LeftVelocity,m_RightVelocity,difference);

	if (m_90DegreeTurnValve==false)
	{
		Ship_2D *ship_rw=const_cast<Ship_2D *>(&ship); //Grrr this is not a perfect solution :(
		UI_Controller* controller=ship_rw->GetController()->GetUIController_RW();
		if (controller)
		{
			if ((m_LeftXAxis>Tank_Steering_XAxisSignal_Threshold)||(m_RightXAxis>Tank_Steering_XAxisSignal_Threshold))
			{
				m_90DegreeTurnValve=true;
				controller->Turn_90R();
			}
			else if ((m_LeftXAxis<-Tank_Steering_XAxisSignal_Threshold)||(m_RightXAxis<-Tank_Steering_XAxisSignal_Threshold))
			{
				m_90DegreeTurnValve=true;
				controller->Turn_90L();
			}
		}
	}
	else if ((fabs(m_LeftXAxis)<Tank_Steering_XAxisSignal_MinThreshold)&&(fabs(m_RightXAxis)<Tank_Steering_XAxisSignal_MinThreshold))
		m_90DegreeTurnValve=false;
}

void Tank_Steering::Joystick_SetLeftVelocity(double Velocity)
{
	if (!m_AreControlsDisabled)
		m_LeftVelocity=Velocity;
	else
		m_LeftVelocity=0.0;
}
void Tank_Steering::Joystick_SetRightVelocity(double Velocity)
{
	if (!m_AreControlsDisabled)
		m_RightVelocity=Velocity;
	else
		m_RightVelocity=0.0;
}

void Tank_Steering::Joystick_SetLeft_XAxis(double Value)
{
	if (!m_AreControlsDisabled)
	{
		m_LeftXAxis=Value;
	}
	else
		m_LeftXAxis=0.0;
}

void Tank_Steering::Joystick_SetRight_XAxis(double Value)
{
	if (!m_AreControlsDisabled)
	{
		m_RightXAxis=Value;
	}
	else
		m_RightXAxis=0.0;
}

void Tank_Steering::BindAdditionalEventControls(bool Bind,Base::EventMap *em,IEvent::HandlerList &ehl)
{
	if (Bind)
	{
		m_StraightDeadZone_Tolerance=m_pParent->GetShipProperties().Get_ShipControls().GetLUA_ShipControls_Props().TankSteering_Tolerance;
		em->EventValue_Map["Joystick_SetLeftVelocity"].Subscribe(ehl,*this, &Tank_Steering::Joystick_SetLeftVelocity);
		em->EventValue_Map["Joystick_SetRightVelocity"].Subscribe(ehl,*this, &Tank_Steering::Joystick_SetRightVelocity);
		em->EventValue_Map["Joystick_SetLeft_XAxis"].Subscribe(ehl,*this, &Tank_Steering::Joystick_SetLeft_XAxis);
		em->EventValue_Map["Joystick_SetRight_XAxis"].Subscribe(ehl,*this, &Tank_Steering::Joystick_SetRight_XAxis);
	}
	else
	{
		em->EventValue_Map["Joystick_SetLeftVelocity"].Remove(*this, &Tank_Steering::Joystick_SetLeftVelocity);
		em->EventValue_Map["Joystick_SetRightVelocity"].Remove(*this, &Tank_Steering::Joystick_SetRightVelocity);
		em->EventValue_Map["Joystick_SetLeft_XAxis"].Remove(*this, &Tank_Steering::Joystick_SetLeft_XAxis);
		em->EventValue_Map["Joystick_SetRight_XAxis"].Remove(*this, &Tank_Steering::Joystick_SetRight_XAxis);
	}
}

  /***********************************************************************************************************************************/
 /*														AI_Base_Controller															*/
/***********************************************************************************************************************************/

AI_Base_Controller::AI_Base_Controller(Ship_2D &ship) : m_Goal(NULL),m_ship(ship),m_UI_Controller(NULL)
{
}

void AI_Base_Controller::UpdateController(double dTime_s)
{
	// Is the player controlling this ship, or is the AI?
	if (m_UI_Controller)
	{
		// The player is controlling with a UI, tell my AI_Reaction to reset so it starts over next time
		m_UI_Controller->UpdateController(dTime_s);
	}
	else
	{
		//Attempt to update a goal (if one exists)
		if (m_Goal)
			m_Goal->Process(dTime_s);
	}
}

void AI_Base_Controller::UpdateUI(double dTime_s)
{
	//UpdateTargetLeadPoint();
	if (m_UI_Controller)
		m_UI_Controller->UpdateUI(dTime_s);
}

bool AI_Base_Controller::Try_SetUIController(UI_Controller *controller)
{
	//So far we have had access to all.  We may want to derive some logic here or establish it at this level (to be determined). 
	m_UI_Controller=controller;
	return true;
}

void AI_Base_Controller::DriveToLocation(Vec2d TrajectoryPoint,Vec2d PositionPoint, double power, double dTime_s,Vec2d* matchVel,bool LockOrientation)
{
	//Supposedly in some compilers _isnan should be available, but isn't defined in math.h... Oh well I don't need this overhead anyhow
	#ifdef WIN32
	if (	_isnan(TrajectoryPoint[0]) ||
			_isnan(TrajectoryPoint[1]) ||
			_isnan(PositionPoint[0]) ||
			_isnan(PositionPoint[1]) ||
			_isnan(power) ||
			_isnan(dTime_s) ||
			(matchVel && (
				_isnan((*matchVel)[0]) ||
				_isnan((*matchVel)[1]))))
	{
		printf("TrajectoryPoint = (%f,%f)\n", TrajectoryPoint[0], TrajectoryPoint[1]);
		printf("PositionPoint = (%f,%f)\n", PositionPoint[0], PositionPoint[1]);
		if (matchVel)
			printf("matchVel = (%f,%f)\n", (*matchVel)[0], (*matchVel)[1] );
		printf("dTime_s = %f, power = %f\n", dTime_s, power);
		assert(false);
	}
	#endif

	Vec2d VectorOffset=TrajectoryPoint-m_ship.GetPos_m();

	if (!LockOrientation)
	{
		//I have kept the older method for reference... both will work identical, but the later avoids duplicate distance work
		#if 0
		double AngularDistance=m_ship.m_IntendedOrientationPhysics.ComputeAngularDistance(VectorOffset);
		//printf("\r %f          ",RAD_2_DEG(AngularDistance));
		m_ship.SetCurrentAngularAcceleration(-AngularDistance,false);
		#else
		double lookDir_radians= atan2(VectorOffset[0],VectorOffset[1]);

		//if (!m_ship.CanStrafe())
		//{
		//	SmartDashboard::PutNumber("TestDirection",RAD_2_DEG(lookDir_radians));
		//	SmartDashboard::PutNumber("TestDirection_HalfPi",RAD_2_DEG(NormalizeRotation_HalfPi(lookDir_radians)));
		//	const double PositionDistance=(PositionPoint-m_ship.GetPos_m()).length();
		//	SmartDashboard::PutNumber("TestDistance",Meters2Feet(PositionDistance));
		//}

		if (m_ship.CanStrafe())
			m_ship.SetIntendedOrientation(lookDir_radians);
		else
		{
			//evaluate delta offset to see if we want to use the reverse absolute position
			double OrientationDelta;
			{
				using namespace std;
				const double current_Att_ABS=m_ship.GetAtt_r() + Pi;
				const double intended_Att_ABS=lookDir_radians + Pi;
				const double Begin=min(current_Att_ABS,intended_Att_ABS);
				const double End=max(current_Att_ABS,intended_Att_ABS);
				const double NormalDelta=End-Begin;			//normal range  -------BxxxxxE-------
				const double InvertedDelta=Begin+(Pi2-End);	//inverted range  xxxxB---------Exxxx
				OrientationDelta=min(NormalDelta,InvertedDelta);
			}
			double orientation_to_use=lookDir_radians;
			//Note the condition to flip has a little bit of extra tolerance to avoid a excessive flipping back and forth
			//We simply multiply the half pi by a scalar
			if (fabs(OrientationDelta)>(PI_2*1.2))
				orientation_to_use=NormalizeRotation2(lookDir_radians + Pi);
			m_ship.SetIntendedOrientation(orientation_to_use);
		}
		#endif
	}

	//first negotiate the max speed given the power
	double MaxSpeed=m_ship.ENGAGED_MAX_SPEED;
	double ScaledSpeed=MaxSpeed;
	{
		if ((power >= 0.0) && (power <= 1.0))
		{
			//Now to compute the speed based on MAX (Note it is up to the designer to make the power smaller in tight turns
			ScaledSpeed= MaxSpeed * power;
			//DEBUG_AUTO_PILOT_SPEED("\rRamora Speeds: Max=%4.1f, Power = %3.1f, Curr = %3.1f",MaxSpeed, power, m_ship.m_Physics.GetLinearVelocity().length());
		}
		else if (power>1.0)
			SetShipVelocity(MIN(power, MaxSpeed));
	}

	if (matchVel)
	{
		VectorOffset=PositionPoint-m_ship.GetPos_m();
		//Vec2d LocalVectorOffset(m_ship.GetAtt_quat().conj() * VectorOffset);
		Vec2d LocalVectorOffset=GlobalToLocal(m_ship.GetAtt_r(),VectorOffset);
		//Vec2d LocalMatchVel(m_ship.GetAtt_quat().conj() * (*matchVel));
		Vec2d LocalMatchVel=GlobalToLocal(m_ship.GetAtt_r(),*matchVel);

		const Vec2d ForceDegradeScalar=m_ship.Get_DriveTo_ForceDegradeScalar();
		Vec2d ForceRestraintPositive(m_ship.MaxAccelRight*m_ship.Mass*ForceDegradeScalar[0],m_ship.m_ShipProps.GetMaxAccelForward()*m_ship.Mass*ForceDegradeScalar[1]);
		Vec2d ForceRestraintNegative(m_ship.MaxAccelLeft*m_ship.Mass*ForceDegradeScalar[0],m_ship.m_ShipProps.GetMaxAccelReverse()*m_ship.Mass*ForceDegradeScalar[1]);
		//Note: it is possible to overflow in extreme distances, if we challenge this then I should have an overflow check in physics
		Vec2d LocalVelocity=m_ship.m_Physics.GetVelocityFromDistance_Linear(LocalVectorOffset,ForceRestraintPositive,ForceRestraintNegative,dTime_s, LocalMatchVel);

		//The logic here should make use of making coordinated turns anytime the forward/reverse velocity has a greater distance than the sides or up/down.
		//Usually if the trajectory point is the same as the position point it will perform coordinated turns most of the time while the nose is pointing
		//towards its goal.  If the nose trajectory is different it may well indeed use the strafing technique more so.

		if (!m_ship.CanStrafe() || (fabs(LocalVelocity[0])<fabs(LocalVelocity[1])))
		{
			//This first technique only works with the forward and partial reverse thrusters (may be useful for some ships)
			//Note: Even though this controls forward and reverse thrusters, the strafe thrusters are still working implicitly to correct turn velocity

			//Now we simply use the positive forward thruster 
			if (LocalVelocity[1]>0.0)  //only forward not reverse...
				SetShipVelocity(MIN(LocalVelocity[1],ScaledSpeed));
			else
				SetShipVelocity(MAX(LocalVelocity[1],-ScaledSpeed));  //Fortunately the ships do not go in reverse that much  :)
		}
		
		else
		{  //This technique makes use of strafe thrusters.  (Currently we can do coordinated turns with this)
			//It is useful for certain situations.  One thing is for sure, it can get the ship
			//to a point more efficiently than the above method, which may be useful for an advanced tactic.
			//Vec2d GlobalVelocity(m_ship.GetAtt_quat() * LocalVelocity); 
			Vec2d GlobalVelocity=LocalToGlobal(m_ship.GetAtt_r(),LocalVelocity); 
			//now to cap off the velocity speeds
			for (size_t i=0;i<2;i++)
			{
				if (GlobalVelocity[i]>ScaledSpeed)
					GlobalVelocity[i]=ScaledSpeed;
				else if (GlobalVelocity[i]<-ScaledSpeed)
					GlobalVelocity[i]=-ScaledSpeed;
			}
			//Ideally GetForceFromVelocity could work with local orientation for FlightDynmic types, but for now we convert
			Vec2d GlobalForce(m_ship.m_Physics.GetForceFromVelocity(GlobalVelocity,dTime_s));
			//Vec2d LocalForce(m_ship.GetAtt_quat().conj() * GlobalForce);
			Vec2d LocalForce=GlobalToLocal(m_ship.GetAtt_r(),GlobalForce); //First get the local force
			//Now to fire all the thrusters given the acceleration
			m_ship.SetCurrentLinearAcceleration(LocalForce/m_ship.Mass);
		}
	}
	else
		SetShipVelocity(ScaledSpeed);
}

  /***********************************************************************************************************************************/
 /*												Goal_Ship_RotateToPosition															*/
/***********************************************************************************************************************************/

Goal_Ship_RotateToPosition::Goal_Ship_RotateToPosition(AI_Base_Controller *controller, double Heading) : m_Controller(controller), m_Heading(Heading),
	m_ship(controller->GetShip()),m_Terminate(false)
{
	m_Status=eInactive;
}
Goal_Ship_RotateToPosition::~Goal_Ship_RotateToPosition()
{
	Terminate(); //more for completion
}

void Goal_Ship_RotateToPosition::Activate() 
{
	m_Status=eActive;
	//During the activation we'll set the requested intended orientation
	m_Controller->SetIntendedOrientation(m_Heading);
}

Goal::Goal_Status Goal_Ship_RotateToPosition::Process(double dTime_s)
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
		if (m_ship.GetIntendedOrientation()==m_Heading)
		{
			double rotation_delta=m_ship.GetAtt_r()-m_Heading;
			NormalizeRotation(rotation_delta);
			//TODO check IsStuck for failed case
			if (fabs(rotation_delta)<m_ship.GetShipProperties().GetShipProps().Rotation_Tolerance)
				m_Status=eCompleted;
		}
		else
			m_Status=eFailed;  //Some thing else took control of the ship
	}
	return m_Status;
}


  /***********************************************************************************************************************************/
 /*												Goal_Ship_RotateToRelativePosition													*/
/***********************************************************************************************************************************/

void Goal_Ship_RotateToRelativePosition::Activate()
{
	m_Heading+=m_ship.GetAtt_r();
	__super::Activate();
}


  /***********************************************************************************************************************************/
 /*												Goal_Ship_MoveToPosition															*/
/***********************************************************************************************************************************/

Goal_Ship_MoveToPosition::Goal_Ship_MoveToPosition(AI_Base_Controller *controller,const WayPoint &waypoint,bool UseSafeStop,
	bool LockOrientation,double safestop_tolerance) : m_Point(waypoint), m_Controller(controller),
	m_ship(controller->GetShip()),m_SafeStopTolerance(safestop_tolerance),m_Terminate(false),m_UseSafeStop(UseSafeStop),m_LockOrientation(LockOrientation)
{
	m_TrajectoryPoint=waypoint.Position;  //set it for default
	m_Status=eInactive;
}
Goal_Ship_MoveToPosition::~Goal_Ship_MoveToPosition()
{
	Terminate(); //more for completion
}

void Goal_Ship_MoveToPosition::SetTrajectoryPoint(const Vec2D &TrajectoryPoint)
{
	m_TrajectoryPoint=TrajectoryPoint;
}

void Goal_Ship_MoveToPosition::Activate() 
{
	m_Status=eActive;
}

bool Goal_Ship_MoveToPosition::HitWayPoint()
{
	// Base a tolerance2 for how close we want to get to the way point based on the current velocity,
	// within a second of reaching the way point, just move to the next one
	//Note for FRC... moving at 2mps it will come within an inch of its point with this tolerance
	double tolerance2 = m_UseSafeStop ? m_SafeStopTolerance : (m_ship.GetPhysics().GetLinearVelocity().length() * 1.0) + 0.1; // (will keep it within one meter even if not moving)
	Vec2d currPos = m_ship.GetPos_m();
	double position_delta=(m_Point.Position-currPos).length();
	bool ret=position_delta<tolerance2;
	#if 0
	printf("\r%f        ",position_delta);
	if (ret)
		printf("completed %f\n",position_delta);
	#endif
	return ret;
}

Goal::Goal_Status Goal_Ship_MoveToPosition::Process(double dTime_s)
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
		//TODO check IsStuck for failed case
		if (!HitWayPoint())
		{
			Vec2d Temp(0,0);
			m_Controller->DriveToLocation(m_TrajectoryPoint , m_Point.Position, m_Point.Power, dTime_s,m_UseSafeStop? &Temp:NULL,m_LockOrientation);
		}
		else
		{
			//for now just stop, but we may want to have a way to identify if we are in a series, perhaps the derived class overrides this and can be more intelligent
			m_Controller->SetShipVelocity(0.0);
			m_Status=eCompleted;
		}
	}
	return m_Status;
}
void Goal_Ship_MoveToPosition::Terminate()
{
	//TODO this may be an inline check
	m_Terminate=true;
}

  /***********************************************************************************************************************************/
 /*												Goal_Ship_MoveToRelativePosition													*/
/***********************************************************************************************************************************/

void Goal_Ship_MoveToRelativePosition::Activate()
{
	//Construct a way point
	WayPoint wp=m_Point;  //set up all the other fields
	const Vec2d &pos=m_ship.GetPos_m();

	const Vec2d Local_GoalTarget=m_Point.Position;

	const Vec2d Global_GoalTarget=LocalToGlobal(m_ship.GetAtt_r(),Local_GoalTarget);
	wp.Position=Global_GoalTarget+pos;
	//set the trajectory point
	double lookDir_radians= atan2(Local_GoalTarget[0],Local_GoalTarget[1]);
	const Vec2d LocalTrajectoryOffset(sin(lookDir_radians),cos(lookDir_radians));
	const Vec2d  GlobalTrajectoryOffset=LocalToGlobal(m_ship.GetAtt_r(),LocalTrajectoryOffset);
	SetTrajectoryPoint(wp.Position+GlobalTrajectoryOffset);
	m_Point=wp;  //This should be a one-time assignment
	__super::Activate();
}

  /***********************************************************************************************************************************/
 /*													Goal_Ship_FollowPath															*/
/***********************************************************************************************************************************/

Goal_Ship_FollowPath::Goal_Ship_FollowPath(AI_Base_Controller *controller,std::list<WayPoint> path,bool LoopMode,bool UseSafeStop) : 
	m_Controller(controller),m_Path(path),m_PathCopy(path),m_LoopMode(LoopMode),m_UseSafeStop(UseSafeStop)
{
	m_Status=eInactive;
}
void Goal_Ship_FollowPath::Activate()
{
	RemoveAllSubgoals();
	if (!m_Path.empty())
	{
		m_Status=eActive;
		WayPoint point=m_Path.front();
		m_Path.pop_front();
		AddSubgoal(new Goal_Ship_MoveToPosition(m_Controller,point,m_UseSafeStop));
	}
	else
		m_Status=eFailed;
}

Goal::Goal_Status Goal_Ship_FollowPath::Process(double dTime_s)
{
	ActivateIfInactive();
	if (m_Status==eActive)
	{
		m_Status=ProcessSubgoals(dTime_s);
		if (m_Status==eCompleted)
		{
			if (m_Path.empty()&&(m_LoopMode))
					m_Path=m_PathCopy;
			if (!m_Path.empty())
				Activate();
		}
	}
	return m_Status;
}

void Goal_Ship_FollowPath::Terminate()
{
}

  /***********************************************************************************************************************************/
 /*													Goal_Ship_FollowShip															*/
/***********************************************************************************************************************************/

void Goal_Ship_FollowShip::SetRelPosition(const Vec2d &RelPosition)  
{
	m_RelPosition=RelPosition;
	m_TrajectoryPosition=RelPosition;
	m_TrajectoryPosition[1] += m_TrajectoryPosition_ForwardOffset;	// Just point forward
}

Goal_Ship_FollowShip::Goal_Ship_FollowShip(AI_Base_Controller *controller,const Ship_2D &Followship,const Vec2d &RelPosition,double Trajectory_ForwardOffset) : 
	m_Controller(controller),m_TrajectoryPosition_ForwardOffset(Trajectory_ForwardOffset),m_Followship(Followship),
	m_ship(controller->GetShip()),m_Terminate(false)
{
	SetRelPosition(RelPosition);
	m_Status=eInactive;
}
Goal_Ship_FollowShip::~Goal_Ship_FollowShip()
{
	Terminate(); //more for completion
}

void Goal_Ship_FollowShip::Activate() 
{
	m_Status=eActive;
}

Goal::Goal_Status Goal_Ship_FollowShip::Process(double dTime_s)
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
		//TODO add these methods to my ship
		//if (followMe.Ship->IsShowing() && !followMe.Ship->IsBeingDestroyed())
		if (true)
		{
			// This is the "correct" offset position
			Vec2d globalGoalPositionPoint;
			Vec2d globalGoalTrajectoryPoint;
			Vec2d globalGoalVelocity;
			globalGoalPositionPoint=globalGoalTrajectoryPoint=m_Followship.GetPos_m();
			globalGoalVelocity = m_Followship.GetPhysics().GetLinearVelocity();


			//globalGoalPositionPoint+=followMe.Ship->GetAtt_quat()*followMe.RelPosition;
			globalGoalPositionPoint+=LocalToGlobal(m_Followship.GetAtt_r(),m_RelPosition);
			//globalGoalTrajectoryPoint  +=followMe.Ship->GetAtt_quat()*followMe.TrajectoryPosition;
			globalGoalTrajectoryPoint+=LocalToGlobal(m_Followship.GetAtt_r(),m_TrajectoryPosition);

			// The globalGoalGlobalOrient position is either in front so we can turn to it
			// AND/OR it is far enough away where we SHOULD make a heavy turn into it.
			//UpdateIntendedLocation(globalGoalTrajectoryPoint,globalGoalPositionPoint, 1.0, &globalGoalVelocity);
			m_Controller->DriveToLocation(globalGoalTrajectoryPoint, globalGoalPositionPoint, 1.0, dTime_s,&globalGoalVelocity);
		}
		else
		{
			// Just drive straight so we can deal with avoidance
			Vec2d globalGoalPositionPoint = m_Followship.GetPos_m() + m_Followship.GetPhysics().GetLinearVelocity();
			//UpdateIntendedLocation(globalGoalPositionPoint,globalGoalPositionPoint, 1.0, NULL);
			m_Controller->DriveToLocation(globalGoalPositionPoint, globalGoalPositionPoint, 1.0, dTime_s, NULL);
			m_Status=eFailed;
		}
	}
	return m_Status;
}
void Goal_Ship_FollowShip::Terminate()
{
	//TODO this may be an inline check
	m_Terminate=true;
}

  /***********************************************************************************************************************************/
 /*															Goal_Wait																*/
/***********************************************************************************************************************************/

Goal_Wait::Goal_Wait(double seconds) : m_TimeToWait(seconds)
{
	m_Status=eInactive;
}

void Goal_Wait::Activate()
{
	m_Status=eActive; 
	m_TimeAccrued=0.0;
}
Goal::Goal_Status Goal_Wait::Process(double dTime_s)
{
	ActivateIfInactive();
	m_TimeAccrued+=dTime_s;
	if (m_TimeAccrued>m_TimeToWait)
		m_Status=eCompleted;
	return m_Status;
}
void Goal_Wait::Terminate()
{
	m_Status=eInactive;
}


  /***********************************************************************************************************************************/
 /*													Goal_NotifyWhenComplete															*/
/***********************************************************************************************************************************/


Goal_NotifyWhenComplete::Goal_NotifyWhenComplete(EventMap &em,const char *EventName,const char *FailedEventName) : m_EventName(EventName),m_EventMap(em)
{
	m_Status=eInactive;
	if (FailedEventName)
		m_FailedEventName=FailedEventName;
}

void Goal_NotifyWhenComplete::Activate()
{
	m_Status=eActive; 
}

Goal::Goal_Status Goal_NotifyWhenComplete::Process(double dTime_s)
{
	//Client will activate
	if (m_Status==eInactive)
		return m_Status;

	if (m_Status==eActive)
	{
		m_Status=ProcessSubgoals(dTime_s);
		if (m_Status==eCompleted)
		{
			m_EventMap.Event_Map[m_EventName].Fire(); //Fire the event
			Terminate();
		}
	}
	else if (m_Status==eFailed)
	{
		if (m_FailedEventName[0]!=0)
			m_EventMap.Event_Map[m_FailedEventName].Fire(); //Fire the event
		Terminate();
	}
	return m_Status;
}

void Goal_NotifyWhenComplete::Terminate()
{
	//ensure its all clean
	RemoveAllSubgoals();
	m_Status=eInactive; //make this inactive
}
