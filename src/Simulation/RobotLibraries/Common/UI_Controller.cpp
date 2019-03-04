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
#include "Ship.h"
#include "AI_Base_Controller.h"
#include "../Base/Joystick.h"
#include "../Base/JoystickBinder.h"
#include "UI_Controller.h"
#include "Debug.h"
#include "SmartDashboard.h"

#undef __EnableTestKeys__

#ifdef Robot_TesterCode
using namespace Robot_Tester;
using namespace GG_Framework::Base;
using namespace GG_Framework::UI;
using namespace osg;
bool Robot_Tester::g_UseMouse=false;
const double Pi=PI;
const double Pi2=PI * 2.0;
#else
using namespace Framework::Base;
using namespace Framework::UI;
#endif

const double Half_Pi=M_PI/2.0;

#ifdef Robot_TesterCode
  /***************************************************************************************************************/
 /*												Mouse_ShipDriver												*/
/***************************************************************************************************************/

Mouse_ShipDriver::Mouse_ShipDriver(Ship_2D& ship,UI_Controller *parent, unsigned avgFrames) : 
	m_ship(ship),m_ParentUI_Controller(parent), m_mousePosHist(NULL), m_avgFrames(avgFrames), m_currFrame(0), m_mouseRoll(false)
{
	//GG_Framework::UI::KeyboardMouse_CB &kbm = GG_Framework::UI::MainWindow::GetMainWindow()->GetKeyboard_Mouse();	
	//kbm.AddKeyBindingR(true, "Ship.MouseRoll", osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON);

	// Set up the Mouse Control to drive
	Entity2D::EventMap* em = m_ship.GetEventMap();
	em->KBM_Events.MouseMove.Subscribe(ehl, *this, &Mouse_ShipDriver::OnMouseMove);
	//em->EventOnOff_Map["Ship.MouseRoll"].Subscribe(ehl, *this, &Mouse_ShipDriver::OnMouseRoll);

	if (m_avgFrames)
	{
		m_mousePosHist = new Vec2f[m_avgFrames];
	}
}
//////////////////////////////////////////////////////////////////////////

void Mouse_ShipDriver::OnMouseMove(float mx, float my)
{
	//Rick keep this here so I can debug code... thanks
	//Ideally when the window loses focus we should release control of the mouse
	if (!g_UseMouse)
	 return;

	// Watch for the window being too small.  We will also need the width and height
	int x,y;
	unsigned w,h;
	GG_Framework::UI::MainWindow::GetMainWindow()->GetWindowRectangle(x,y,w,h);
	if ((w < 2) || (h < 2))
		return;

	// We want to use the physical distance in pixels to normalize the distance the mouse moves
	// and to avoid the re-centering effect that gives a small amount of pixel deviation
	float pixelX, pixelY;
	if (!GG_Framework::UI::MainWindow::GetMainWindow()->ComputePixelCoords(mx, my, pixelX, pixelY))
		return;

	// We are assuming that the mouse was set to its 0,0 position (center screen) Before every call
	float dX = pixelX - (float)x - ((float)w/2.0f);
	float dY = pixelY - (float)y - ((float)h/2.0f);

	// Remember this for the next time, if larger than what we have
	if ((dX > 1.5f) || (dX < -1.5f))  // Watch for very small amounts of mouse drift
	{
		if (fabs(dX) > fabs(m_lastMousePos[0]))
			m_lastMousePos[0] = dX;
	}
	if ((dY > 1.5f) || (dY < -1.5f))  // Watch for very small amounts of mouse drift
	{
		if (fabs(dY) > fabs(m_lastMousePos[1]))
			m_lastMousePos[1] = dY;
	}

	// Reset back to 0,0 for next time
	GG_Framework::UI::MainWindow::GetMainWindow()->PositionPointer(0.0f,0.0f);
}
//////////////////////////////////////////////////////////////////////////

void Mouse_ShipDriver::DriveShip()
{
	float dX = 0.0f;
	float dY = 0.0f;
	if (m_mousePosHist)
	{
		// Store from this frame
		if (m_currFrame >= m_avgFrames)
			m_currFrame = 0;
		m_mousePosHist[m_currFrame] = m_lastMousePos;
		++m_currFrame;

		// Get the average
		for (unsigned i=0; i < m_avgFrames; ++i)
		{
			dX += m_mousePosHist[i][0];
			dY += m_mousePosHist[i][1];
		}
		dX /= (float)m_avgFrames;
		dY /= (float)m_avgFrames;
	}
	else
	{
		// We are not using averages, just use this value
		dX = m_lastMousePos[0];
		dY = m_lastMousePos[1];
	}
	// Reset for next time
	m_lastMousePos = Vec2f(0.0f, 0.0f);

	// Will be used eventually for sensitivity and mouse flip, store in script, etc.
	static const float x_coeff = 0.004f;
	static const float y_coeff = -0.004f;
	static const float roll_coeff = 0.01f;

	// Finally Turn the Heading or Pitch (or roll if using the rt mouse button
	if (m_mouseRoll)
	{
		// Use this roll versus the scroll wheel
		float dR = dY - dX;

		//! \todo JAMES: here is the funky roll
		//if (dR != 0.0f)
		//	m_ParentUI_Controller->Mouse_Roll(dR*roll_coeff);
	}
	else
	{
		if (dX != 0.0f)
			m_ParentUI_Controller->Mouse_Turn(dX*x_coeff);
		//if (dY != 0.0f)
		//	m_ParentUI_Controller->Mouse_Pitch(dY*y_coeff);
	}
}                                                      
//////////////////////////////////////////////////////////////////////////

#endif

  /***************************************************************************************************************/
 /*										UI_Controller::FieldCentricDrive										*/
/***************************************************************************************************************/

UI_Controller::FieldCentricDrive::FieldCentricDrive(UI_Controller *pParent) : m_pParent(pParent),m_PosX(0.0),m_PosY(0.0),m_HeadingLock(0.0),
	m_XAxisEnableThreshold(0.4),m_FieldCentricDrive_Mode(false)
{

}

void UI_Controller::FieldCentricDrive::TimeChange(double dTime_s)
{
	//To enter the field centric mode there has to have been an initial strafe to activate it or bind button to manually activate... 
	//once activated it can stay in this mode until the rotation is used
	//once rotation is used it will stay disabled until the next strafe is on... and so on... this ensures normal mode is maintained until the user explicitly wants
	//to switch modes
	if (!IsZero(m_pParent->m_Ship_JoyMouse_rotAcc_rad_s))
		m_FieldCentricDrive_Mode=false;
	else if (fabs(m_PosX)>m_XAxisEnableThreshold)
		m_FieldCentricDrive_Mode=true;

	SmartDashboard::PutBoolean("FieldCentricDrive_Mode",m_FieldCentricDrive_Mode);
	if (m_FieldCentricDrive_Mode)
	{
		const double YValue=m_PosY;
		double Value=m_PosX;

		//This version limits to 90 degrees... so any down motion will be treated like up motion
		//const double theta = atan2(Value,fabs(YValue));
		const double theta = atan2(Value,YValue);
		//Find the magnitude
		const double magnitude = sqrt(((Value * Value) + (YValue * YValue)));
		//SmartDashboard::PutNumber("TestMagnitude",magnitude);
		const double lookDir_radians=NormalizeRotation2(theta + m_HeadingLock);
		//evaluate delta offset to see if we want to use the reverse absolute position
		Ship_2D &m_ship=*m_pParent->m_ship;
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
			//SmartDashboard::PutNumber("TestOrientationDelta",RAD_2_DEG(OrientationDelta));
		}
		double orientation_to_use=lookDir_radians;
		double NormalizedVelocity=fabs(cos(fabs(OrientationDelta))*magnitude);
		//Note the condition to flip has a little bit of extra tolerance to avoid a excessive flipping back and forth
		//We simply multiply the half pi by a scalar
		if (fabs(OrientationDelta)>(PI_2*1.2))
		{
			orientation_to_use=NormalizeRotation2(lookDir_radians + Pi);
			NormalizedVelocity=-NormalizedVelocity;
		}
		//SmartDashboard::PutNumber("TestNormalizedVelocity",NormalizedVelocity);
		if (magnitude>0.4)
		{
			//SmartDashboard::PutNumber("TestAngle",RAD_2_DEG(orientation_to_use));
			m_ship.SetIntendedOrientation(orientation_to_use);
			m_pParent->m_Ship_UseHeadingSpeed=false;
		}
		//m_pParent->m_Ship_UseHeadingSpeed=fabs(OrientationDelta)>DEG_2_RAD(5.0)?false:true;
		m_pParent->Quadrant_SetCurrentSpeed(NormalizedVelocity);
	}
	else
	{
		//rotation used... work with Y axis like before
		m_HeadingLock=NormalizeRotation2(m_pParent->m_ship->GetAtt_r());
		m_pParent->Quadrant_SetCurrentSpeed(m_PosY);
	}
}

void UI_Controller::FieldCentricDrive::FieldCentricDrive_Mode_Enable()
{
	m_HeadingLock=NormalizeRotation2(m_pParent->m_ship->GetAtt_r());
	m_FieldCentricDrive_Mode=true;
}

void UI_Controller::FieldCentricDrive::FieldCentricDrive_Mode_Enable(double Value) 
{
	if (Value > m_XAxisEnableThreshold)
	{
		if (!m_FieldCentricDrive_Mode)
			FieldCentricDrive_Mode_Enable();
	}
}

void UI_Controller::FieldCentricDrive::BindAdditionalEventControls(bool Bind,Base::EventMap *em,IEvent::HandlerList &ehl)
{
	if (Bind)
	{
		m_XAxisEnableThreshold=m_pParent->m_ship->GetShipProperties().Get_ShipControls().GetLUA_ShipControls_Props().FieldCentricDrive_XAxisEnableThreshold;
		em->EventValue_Map["Joystick_FieldCentric_XAxis"].Subscribe(ehl,*this, &FieldCentricDrive::UpdatePosX);
		em->EventValue_Map["Joystick_FieldCentric_YAxis"].Subscribe(ehl,*this, &FieldCentricDrive::UpdatePosY);
		em->EventValue_Map["FieldCentric_EnableValue"].Subscribe(ehl,*this, &FieldCentricDrive::FieldCentricDrive_Mode_Enable);
		em->Event_Map["FieldCentric_Enable"].Subscribe(ehl,*this, &FieldCentricDrive::FieldCentricDrive_Mode_Enable);
	}
	else
	{
		em->EventValue_Map["Joystick_FieldCentric_XAxis"].Remove(*this, &FieldCentricDrive::UpdatePosX);
		em->EventValue_Map["Joystick_FieldCentric_YAxis"].Remove(*this, &FieldCentricDrive::UpdatePosY);
		em->EventValue_Map["FieldCentric_EnableValue"].Remove(*this, &FieldCentricDrive::FieldCentricDrive_Mode_Enable);
		em->Event_Map["FieldCentric_Enable"].Remove(*this, &FieldCentricDrive::FieldCentricDrive_Mode_Enable);
	}
}

  /***************************************************************************************************************/
 /*												UI_Controller													*/
/***************************************************************************************************************/

void UI_Controller::Init_AutoPilotControls()
{
}

#ifdef Robot_TesterCode
UI_Controller::UI_Controller(AI_Base_Controller *base_controller,bool AddJoystickDefaults) : 
#else
UI_Controller::UI_Controller(JoyStick_Binder *joy,AI_Base_Controller *base_controller) : 
#endif
	m_Base(NULL),
	#ifdef Robot_TesterCode
	m_mouseDriver(NULL),
	#else
	m_JoyStick_Binder(joy),
	#endif
	m_isControlled(false),m_ShipKeyVelocity(0.0),m_SlideButtonToggle(false),m_CruiseSpeed(0.0),m_YFlipScalar(1.0),
	m_autoPilot(true),m_enableAutoLevelWhenPiloting(false),m_Ship_UseHeadingSpeed(true),m_Test1(false),m_Test2(false),m_IsBeingDestroyed(false),
	m_POVSetValve(false),m_FieldCentricDrive(this)
{
	ResetPos();
	Set_AI_Base_Controller(base_controller); //set up ship (even if we don't have one)
	m_LastSliderTime[0]=m_LastSliderTime[1]=0.0;

	#ifdef Robot_TesterCode
	// Hard code these key bindings at first
	KeyboardMouse_CB &kbm = MainWindow::GetMainWindow()->GetKeyboard_Mouse();	
	JoyStick_Binder &joy = MainWindow::GetMainWindow()->GetJoystick();
	kbm.AddKeyBindingR(true, "Ship.TryFireMainWeapon", osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
	//disabled until it works
	//kbm.AddKeyBindingR(true, "RequestAfterburner", GG_Framework::Base::Key('w', GG_Framework::Base::Key::DBL));
	kbm.AddKeyBindingR(true, "Thrust", GG_Framework::Base::Key('w'));
	kbm.AddKeyBindingR(true, "Brake", 's');
	kbm.AddKeyBindingR(false, "Stop", 'x');
	kbm.AddKeyBindingR(true, "Turn_R", 'd');
	kbm.AddKeyBindingR(true, "Turn_L", 'a');
	kbm.AddKeyBindingR(false, "Turn_180", '`');
	kbm.AddKeyBindingR(false, "UseMouse", '/');
	//I would like to keep this macro case to easily populate my defaults
	#if 0
	//for testing
	kbm.AddKeyBindingR(true, "Test1", 'n');
	kbm.AddKeyBindingR(true, "Test2", 'm');
	#endif
	kbm.AddKeyBindingR(true, "StrafeRight", 'e');
	kbm.AddKeyBindingR(true, "StrafeLeft", 'q');
	kbm.AddKeyBindingR(false, "UserResetPos", ' ');
	//Using g for goal now ;)
	//kbm.AddKeyBindingR(false, "Slide", 'g');
	kbm.AddKeyBindingR(true, "StrafeLeft", osgGA::GUIEventAdapter::KEY_Left);
	kbm.AddKeyBindingR(true, "StrafeRight", osgGA::GUIEventAdapter::KEY_Right);
	kbm.AddKeyBindingR(false, "ToggleAutoPilot", 'z');
	//kbm.AddKeyBindingR(false, "ShowHUD", osgGA::GUIEventAdapter::KEY_F4);

	if (AddJoystickDefaults)
	{
		joy.AddJoy_Button_Default(0,"Ship.TryFireMainWeapon");
		joy.AddJoy_Button_Default(1,"Missile.Launch");
		// We can now use double-tap to fire the afterburners (for when we have them)
		joy.AddJoy_Button_Default(2,"Thrust");
		joy.AddJoy_Button_Default(3,"Brake");
		joy.AddJoy_Analog_Default(JoyStick_Binder::eSlider1,"Joystick_SetCurrentSpeed");
		//These are not assigned by default but can configured to use via xml preferences
		joy.AddJoy_Analog_Default(JoyStick_Binder::eX_Axis,"Analog_Turn",false,1.0,0.01,true);
		//hmmm could use this to thrust
		//joy.AddJoy_Analog_Default(JoyStick_Binder::eY_Axis,"Analog_Pitch",true,1.0,0.01,true);
		joy.AddJoy_Button_Default(6,"Slide",false);
		joy.AddJoy_Analog_Default(JoyStick_Binder::eZ_Rot,"Analog_StrafeRight");
	}
	#endif

	Init_AutoPilotControls();
}


const char *UI_Controller::ExtractControllerElementProperties(Controller_Element_Properties &Element,const char *Eventname,Scripting::Script& script)
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
			JoyStick_Binder::JoyAxis_enum JoyAxis;
			double dJoyAxis;
			err = script.GetField("key", NULL, NULL,&dJoyAxis);
			ASSERT_MSG(!err, err);
			//cast to int first, and then to the enumeration
			JoyAxis=(JoyStick_Binder::JoyAxis_enum)((int)dJoyAxis);
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
		else if (strcmp(sType.c_str(),"joystick_button")==0)
		{
			Element.Type=Controller_Element_Properties::eJoystickButton;
			size_t WhichButton;
			double dWhichButton;
			err = script.GetField("key", NULL, NULL,&dWhichButton);
			ASSERT_MSG(!err, err);
			//cast to int first, and then to the enumeration; The -1 allows for cardinal types (good since we can use numbers written on button)
			WhichButton=(JoyStick_Binder::JoyAxis_enum)((int)dWhichButton-1);
			bool useOnOff;
			err = script.GetField("on_off", NULL, &useOnOff,NULL);
			ASSERT_MSG(!err, err);
			bool dbl_click=false;
			err = script.GetField("dbl", NULL, &dbl_click,NULL); //This one can be blank
			err=NULL;  //don't return an error (assert for rest)

			Controller_Element_Properties::ElementTypeSpecific::ButtonSpecifics_rw &set=Element.Specifics.Button;
			set.WhichButton=WhichButton;
			set.useOnOff=useOnOff;
			set.dbl_click=dbl_click;
			//joy.AddJoy_Button_Default( WhichButton,Eventname,useOnOff,dbl_click,ProductName.c_str());
		}
		else assert(false);
		script.Pop();
	}
	return err;
}

void UI_Controller::Flush_AI_BaseResources()
{
	#ifdef Robot_TesterCode
	if (m_mouseDriver)
	{
		delete m_mouseDriver;
		m_mouseDriver=NULL;
	}
	#endif
	m_ship=NULL; //we don't own this
}

UI_Controller::~UI_Controller()
{
	m_IsBeingDestroyed=true;
	Set_AI_Base_Controller(NULL); //this will unbind the events and flush the AI resources
}

UI::JoyStick_Binder &UI_Controller::GetJoyStickBinder()
{
	#ifdef Robot_TesterCode
	return MainWindow::GetMainWindow()->GetJoystick();
	#else
	return *m_JoyStick_Binder;
	#endif
}

KeyboardMouse_CB *UI_Controller::GetKeyboardBinder()
{
	#ifdef Robot_TesterCode
	return &MainWindow::GetMainWindow()->GetKeyboard_Mouse();
	#else
	return &m_Keyboard;
	#endif
}

void UI_Controller::Set_AI_Base_Controller(AI_Base_Controller *controller)
{
	//destroy all resources associated with the previous ship
	if (m_Base)
	{
		Entity2D_Kind::EventMap* em = m_ship->GetEventMap();
		//disabled until it works
		//em->EventOnOff_Map["RequestAfterburner"].Remove(*this, &UI_Controller::AfterBurner_Thrust);
		em->EventOnOff_Map["Thrust"].Remove(*this, &UI_Controller::Thrust);
		em->EventOnOff_Map["Brake"].Remove(*this, &UI_Controller::Brake);
		em->Event_Map["Stop"].Remove(*this, &UI_Controller::Stop);
		em->EventOnOff_Map["Turn_R"].Remove(*this, &UI_Controller::Turn_R);
		em->EventOnOff_Map["Turn_L"].Remove(*this, &UI_Controller::Turn_L);
		em->Event_Map["Turn_90R"].Remove(*this, &UI_Controller::Turn_90R);
		em->Event_Map["Turn_90L"].Remove(*this, &UI_Controller::Turn_90L);
		em->Event_Map["Turn_180"].Remove(*this, &UI_Controller::Turn_180);
		em->EventOnOff_Map["Turn_180_Hold"].Remove(*this, &UI_Controller::Turn_180_Hold);
		em->Event_Map["FlipY"].Remove(*this, &UI_Controller::FlipY);
		em->EventOnOff_Map["FlipY_Hold"].Remove(*this, &UI_Controller::FlipY_Hold);
		em->Event_Map["UserResetPos"].Remove(*this, &UI_Controller::UserResetPos);
		em->Event_Map["ResetPos"].Remove(*this, &UI_Controller::ResetPos);
		em->Event_Map["Slide"].Remove(*this, &UI_Controller::ToggleSlide);
		em->EventOnOff_Map["SlideHold"].Remove(*this, &UI_Controller::SlideHold);
		em->EventOnOff_Map["StrafeLeft"].Remove(*this, &UI_Controller::StrafeLeft);
		em->EventOnOff_Map["StrafeRight"].Remove(*this, &UI_Controller::StrafeRight);
		em->Event_Map["ToggleAutoPilot"].Remove(*this, &UI_Controller::TryToggleAutoPilot);
		em->EventOnOff_Map["SPAWN"].Remove(*this, &UI_Controller::OnSpawn);
		#ifdef Robot_TesterCode
		em->Event_Map["UseMouse"].Remove(*this, &UI_Controller::UseMouse);
		#endif
		em->EventOnOff_Map["Test1"].Remove(*this, &UI_Controller::Test1);
		em->EventOnOff_Map["Test2"].Remove(*this, &UI_Controller::Test2);
		em->EventValue_Map["BLACKOUT"].Remove(*this, &UI_Controller::BlackoutHandler);

		em->EventValue_Map["POV_Turn"].Remove(*this, &UI_Controller::Ship_Turn90_POV);
		em->EventValue_Map["Analog_Turn"].Remove(*this, &UI_Controller::JoyStick_Ship_Turn);
		em->EventValue_Map["Analog_StrafeRight"].Remove(*this, &UI_Controller::StrafeRight);
		em->EventValue_Map["Analog_Slider_Accel"].Remove(*this, &UI_Controller::Slider_Accel);
		em->EventValue_Map["Joystick_SetCurrentSpeed"].Remove(*this, &UI_Controller::Joystick_SetCurrentSpeed);
		em->EventValue_Map["Joystick_SetCurrentSpeed_2"].Remove(*this, &UI_Controller::Joystick_SetCurrentSpeed_2);
		m_ship->BindAdditionalEventControls(false);
		m_FieldCentricDrive.BindAdditionalEventControls(false,em,ehl);
		if (!m_IsBeingDestroyed)
			m_ship->BindAdditionalUIControls(false,&GetJoyStickBinder(),GetKeyboardBinder());
		Flush_AI_BaseResources();
	}
	m_Base=controller;
	if (m_Base)
	{
		m_ship=&m_Base->m_ship;
		#ifdef Robot_TesterCode
		m_mouseDriver=new Mouse_ShipDriver(*m_ship,this, 3);
		#endif
		Entity2D_Kind::EventMap *em = m_ship->GetEventMap();
		//disabled until it works
		//em->EventOnOff_Map["RequestAfterburner"].Subscribe(ehl, *this, &UI_Controller::AfterBurner_Thrust);
		em->EventOnOff_Map["Thrust"].Subscribe(ehl, *this, &UI_Controller::Thrust);
		em->EventOnOff_Map["Brake"].Subscribe(ehl, *this, &UI_Controller::Brake);
		em->Event_Map["Stop"].Subscribe(ehl, *this, &UI_Controller::Stop);
		em->EventOnOff_Map["Turn_R"].Subscribe(ehl, *this, &UI_Controller::Turn_R);
		em->EventOnOff_Map["Turn_L"].Subscribe(ehl, *this, &UI_Controller::Turn_L);
		em->Event_Map["Turn_90R"].Subscribe(ehl, *this, &UI_Controller::Turn_90R);
		em->Event_Map["Turn_90L"].Subscribe(ehl, *this, &UI_Controller::Turn_90L);
		em->Event_Map["Turn_180"].Subscribe(ehl, *this, &UI_Controller::Turn_180);
		em->EventOnOff_Map["Turn_180_Hold"].Subscribe(ehl, *this, &UI_Controller::Turn_180_Hold);
		em->Event_Map["FlipY"].Subscribe(ehl, *this, &UI_Controller::FlipY);
		em->EventOnOff_Map["FlipY_Hold"].Subscribe(ehl, *this, &UI_Controller::FlipY_Hold);
		em->Event_Map["UserResetPos"].Subscribe(ehl, *this, &UI_Controller::UserResetPos);
		em->Event_Map["ResetPos"].Subscribe(ehl, *this, &UI_Controller::ResetPos);
		em->Event_Map["Slide"].Subscribe(ehl, *this, &UI_Controller::ToggleSlide);
		em->EventOnOff_Map["SlideHold"].Subscribe(ehl, *this, &UI_Controller::SlideHold);
		em->EventOnOff_Map["StrafeLeft"].Subscribe(ehl, *this, &UI_Controller::StrafeLeft);
		em->EventOnOff_Map["StrafeRight"].Subscribe(ehl, *this, &UI_Controller::StrafeRight);
		em->Event_Map["ToggleAutoPilot"].Subscribe(ehl, *this, &UI_Controller::TryToggleAutoPilot);
		em->EventOnOff_Map["SPAWN"].Subscribe(ehl, *this, &UI_Controller::OnSpawn);

		#ifdef Robot_TesterCode
		em->Event_Map["UseMouse"].Subscribe(ehl, *this, &UI_Controller::UseMouse);
		#endif
		em->EventOnOff_Map["Test1"].Subscribe(ehl, *this, &UI_Controller::Test1);
		em->EventOnOff_Map["Test2"].Subscribe(ehl, *this, &UI_Controller::Test2);

		// And a button for the Speed PDCB
		//em->Event_Map["ShowHUD"].Subscribe(m_HUD_UI->ehl, *m_HUD_UI.get(), &HUD_PDCB::ToggleEnabled);

		// Listen for blackout
		em->EventValue_Map["BLACKOUT"].Subscribe(ehl, *this, &UI_Controller::BlackoutHandler);

		em->EventValue_Map["POV_Turn"].Subscribe(ehl,*this, &UI_Controller::Ship_Turn90_POV);
		em->EventValue_Map["Analog_Turn"].Subscribe(ehl,*this, &UI_Controller::JoyStick_Ship_Turn);
		em->EventValue_Map["Analog_StrafeRight"].Subscribe(ehl,*this, &UI_Controller::StrafeRight);
		em->EventValue_Map["Analog_Slider_Accel"].Subscribe(ehl,*this, &UI_Controller::Slider_Accel);
		em->EventValue_Map["Joystick_SetCurrentSpeed"].Subscribe(ehl,*this, &UI_Controller::Joystick_SetCurrentSpeed);
		em->EventValue_Map["Joystick_SetCurrentSpeed_2"].Subscribe(ehl,*this, &UI_Controller::Joystick_SetCurrentSpeed_2);

		// Tell the HUD the name of this ship
		//m_HUD_UI->m_addnText = m_ship->GetName();

		m_ship->BindAdditionalEventControls(true);
		m_FieldCentricDrive.BindAdditionalEventControls(true,em,ehl);
		if (m_JoyStick_Binder)
			m_ship->BindAdditionalUIControls(true,&GetJoyStickBinder(),GetKeyboardBinder());
	}
}

void UI_Controller::Test1(bool on)
{
	m_Test1=on;
}
void UI_Controller::Test2(bool on)
{
	m_Test2=on;
}

#ifdef Robot_TesterCode
void UI_Controller::UseMouse()
{
	g_UseMouse=!g_UseMouse;
}
#endif

void UI_Controller::Ship_AfterBurner_Thrust(bool on)	
{	
	//Note this will happen implicitly
	// Touching the Afterburner always places us back in SImFLight mode
	//m_ship->SetSimFlightMode(true);

	// Set the current requested speed for the ship based on whether we are turning afterburner on or off
	if (on)
		m_ship->FireAfterburner();
	else
	{
		// Set the requested speed to our current speed or at the max engaged speed (Afterburner Brake)
		//Vec2d LocalVelocity(m_ship->GetAtt_quat().conj() * m_ship->GetPhysics().GetLinearVelocity());
		Vec2d LocalVelocity=GlobalToLocal(m_ship->GetAtt_r(),m_ship->GetPhysics().GetLinearVelocity());
		double currSpeed = LocalVelocity[1];
		m_ship->SetRequestedVelocity(MIN(currSpeed, m_ship->GetEngaged_Max_Speed()));
	}
}

void UI_Controller::Ship_Thrust(bool on)
{	
	if (on)
		m_Ship_Keyboard_currAccel[1] = m_ship->GetAccelSpeed();
	else
		m_Ship_Keyboard_currAccel[1] = 0.0;
}

void UI_Controller::Ship_Brake(bool on)
{	
	if (on)
		m_Ship_Keyboard_currAccel[1] = -m_ship->GetBrakeSpeed();
	else
		m_Ship_Keyboard_currAccel[1] = 0.0;
}

void UI_Controller::Ship_Thrust(double Intensity)
{	
	if (fabs(Intensity)<0.001)  //Weed out empty Joystick calls that have no effect
		return;
	m_Ship_JoyMouse_currAccel[1] = m_ship->GetAccelSpeed()*Intensity;
}

void UI_Controller::Ship_Brake(double Intensity)
{	
	if (fabs(Intensity)<0.001)  //Weed out empty Joystick calls that have no effect
		return;
	m_Ship_JoyMouse_currAccel[1] = -m_ship->GetBrakeSpeed()*Intensity;
}

void UI_Controller::Ship_StrafeLeft(double Intensity)	
{		
	if (fabs(Intensity)<0.001)  //Weed out empty Joystick calls that have no effect
		return;
	m_Ship_JoyMouse_currAccel[0]=  -m_ship->GetStrafeSpeed()*Intensity;	
}
void UI_Controller::Ship_StrafeRight(double Intensity)	
{
	if (fabs(Intensity)<0.001)  //Weed out empty Joystick calls that have no effect
		return;
	m_Ship_JoyMouse_currAccel[0]=  m_ship->GetStrafeSpeed()*Intensity;	
}

void UI_Controller::Ship_Turn(double dir,bool UseHeadingSpeed) 
{
	if (fabs(dir)<0.001)  //Weed out empty Joystick calls that have no effect
		return;
	m_Ship_UseHeadingSpeed=UseHeadingSpeed;
	m_Ship_JoyMouse_rotAcc_rad_s=(UseHeadingSpeed?dir*m_ship->GetHeadingSpeed():dir)*m_ship->GetCameraRestraintScaler();
}

#undef __TWEAK180__
void UI_Controller::Ship_Turn(Directions dir)
{
	switch (dir)
	{
		case Dir_None:
		case Dir_Left:
		case Dir_Right:
			m_Ship_Keyboard_rotAcc_rad_s=(double)dir*m_ship->GetHeadingSpeed()*m_ship->GetCameraRestraintScaler();
			m_Ship_UseHeadingSpeed=true;
			break;
		case Dir_90Left:
			m_ship->SetIntendedOrientation(-Half_Pi,false);
			m_Ship_UseHeadingSpeed=false;
			break;
		case Dir_90Right:
			m_ship->SetIntendedOrientation(Half_Pi,false);
			m_Ship_UseHeadingSpeed=false;
			break;
		case Dir_180:
			#if defined( __TWEAK180__) && !defined (__DisableSmartDashboard__)
			struct SetUp { SetUp() {SmartDashboard::PutNumber("Tweak180",180.0);} };
			static SetUp init;
			const double TurnAmount=SmartDashboard::GetNumber("Tweak180");
			m_ship->SetIntendedOrientation(DEG_2_RAD(TurnAmount),false);
			#else
			m_ship->SetIntendedOrientation(Pi,false);
			#endif
			m_Ship_UseHeadingSpeed=false;
			break;
	}
}

void UI_Controller::Turn_RelativeOffset(double value,bool Absolute) 
{
	if (AreControlsDisabled()) return;
	m_ship->SetIntendedOrientation(value,Absolute);
	m_Ship_UseHeadingSpeed=false;
}

void UI_Controller::Ship_Turn90_POV (double value)
{
	//We put the typical case first (save the amount of branching)
	if (value!=-1)
	{
		if (!m_POVSetValve)
		{
			m_POVSetValve=true;
			m_ship->SetIntendedOrientation(DEG_2_RAD(value),false);
			m_Ship_UseHeadingSpeed=false;
		}
	}
	else 
		m_POVSetValve=false;
}

void UI_Controller::Turn_180_Hold(bool on) 
{
	if (AreControlsDisabled()) return; 
	if (on)
		Ship_Turn(Dir_180);
	else
	{
		m_Ship_UseHeadingSpeed=true;
		m_ship->SetCurrentAngularAcceleration(0.0,true);
	}
}

void UI_Controller::FlipY()
{
	m_YFlipScalar=-m_YFlipScalar;
}

void UI_Controller::CancelAllControls()
{
	// Turns off all controls that might be on
	AfterBurner_Thrust(false);
	Thrust(false);
	Brake(false);
	Turn_L(false);
	Turn_R(false);
	StrafeLeft(false);
	StrafeRight(false);
	Test1(false);
	Test2(false);
	TryFireMainWeapon(false);
}

void UI_Controller::TryToggleAutoPilot()
{
	SetAutoPilot(!GetAutoPilot());
}

bool UI_Controller::SetAutoPilot(bool autoPilot)
{
	SmartDashboard::PutBoolean("AutoPilot",autoPilot);
	// Note that Autopilot MUST be on if the ship is not being controlled
	m_autoPilot = autoPilot || !m_isControlled || !m_Base->GetCanUserPilot();

	// When in autopilot, always use mouse POV
	//ToggleMousePOV(m_autoPilot);

	//Since we do not control stabilize we don't manage it here in the controller... we can only write things we can control.  The stabilize rotation should be managed at the lower level
	//unless we really find a compelling reason to need to control it... 
	//  [7/21/2013 Terminator]

	// If we are in auto-pilot, we MUST set simulated flight mode if we have an auto-pilot route
	if (m_autoPilot)
	{
		bool hasAutoPilotRoute = m_Base->HasAutoPilotRoute();
		//m_ship->SetStabilizeRotation(hasAutoPilotRoute);
		m_ship->SetSimFlightMode(hasAutoPilotRoute);
	}
	else
	{
		//m_ship->SetStabilizeRotation(true);
		m_ship->SetSimFlightMode(!m_SlideButtonToggle);
	}

	// When turning on or OFF the auto pilot, stop firing and other actions
	m_ship->CancelAllControls();

	return m_autoPilot;
}

void UI_Controller::UserResetPos()
{
	if (AreControlsDisabled())
		return;
	//TODO see if this call is safe... it was put here as m_Ship_JoyMouse_rotAcc_rad_s would get stuck causing additional movement
	//(when testing swerve robot)... if this causes problems the UI Ship may call it
	ResetPos();
	m_ship->ResetPos();
}

void UI_Controller::ResetPos()
{
	m_Ship_Keyboard_rotAcc_rad_s =	m_Ship_JoyMouse_rotAcc_rad_s = m_ShipKeyVelocity = 0.0;
	m_Ship_Keyboard_currAccel = m_Ship_JoyMouse_currAccel =	Vec2d(0,0);

	//m_HUD_UI->Reset();
	//TODO see where ResetPos is called within the game, as m_Base does not have a check
	if (m_Base)
		m_Base->ResetPos();
}

void UI_Controller::BlackoutHandler(double bl)
{
	// Turn STRAFE on the ship to pull out of the blackout
	if ((bl != 0.0) && (!m_autoPilot))
		m_ship->SetSimFlightMode(false);
}

void UI_Controller::Mouse_Turn(double dir)
{
	Ship_Turn(dir,false);
}

//Note: XTerminator filter = 0.01, and is squared have been transferred to default settings
void UI_Controller::JoyStick_Ship_Turn(double dir)
{
	if (AreControlsDisabled())
		return;

	Ship_Turn(dir);
}

void UI_Controller::Slider_Accel(double Intensity)
{
	if (AreControlsDisabled())
		return;

	if (Intensity>0.0)
		Ship_Thrust(Intensity);
	else if (Intensity<0.0)
		Ship_Brake(-Intensity);
}

void UI_Controller::Quadrant_SetCurrentSpeed(double NormalizedVelocity)
{
	if ((m_Ship_Keyboard_currAccel[1]==0.0)&&(!AreControlsDisabled()))
	{
		if (m_ship->GetAlterTrajectory())
		{
			const double SpeedToUse=m_ship->GetIsAfterBurnerOn()?m_ship->GetMaxSpeed():m_ship->GetEngaged_Max_Speed();
			m_CruiseSpeed+=NormalizedVelocity * SpeedToUse;
			//This one is intended to be used with other axis control since its clean I'm not going to use m_LastSliderTime check
		}
	}
}

void UI_Controller::Joystick_SetCurrentSpeed(double Speed)
{
	if ((m_Ship_Keyboard_currAccel[1]==0.0)&&(!AreControlsDisabled()))
	{
		if (m_ship->GetAlterTrajectory())
		{
			const double SpeedToUse=m_ship->GetIsAfterBurnerOn()?m_ship->GetMaxSpeed():m_ship->GetEngaged_Max_Speed();
			//This works but I really did not like the feel of it
			const double SpeedCalibrated=((Speed/2.0)+0.5)*SpeedToUse;
			m_LastSliderTime[1]=Speed;
			m_CruiseSpeed+=SpeedCalibrated;
		}
		else
			m_Ship_JoyMouse_currAccel[1]=Speed * (Speed>0.0?m_ship->GetAccelSpeed():m_ship->GetBrakeSpeed());
	}
}

void UI_Controller::Joystick_SetCurrentSpeed_2(double Speed)
{
	if ((m_Ship_Keyboard_currAccel[1]==0.0)&&(!AreControlsDisabled()))
	{
		if (m_ship->GetAlterTrajectory())
		{
			const double SpeedToUse=m_ship->GetIsAfterBurnerOn()?m_ship->GetMaxSpeed():m_ship->GetEngaged_Max_Speed();
			const double SpeedCalibrated=Speed*SpeedToUse;
			m_LastSliderTime[1]=Speed;
			m_CruiseSpeed+=SpeedCalibrated;
		}
		else
			m_Ship_JoyMouse_currAccel[1]=Speed * (Speed>0.0?m_ship->GetAccelSpeed():m_ship->GetBrakeSpeed());
	}
}

bool UI_Controller::AreControlsDisabled()
{
	//return (m_autoPilot || !m_ship->IsShowing());
	return (m_autoPilot );
}

void UI_Controller::UpdateController(double dTime_s)
{
	if (AreControlsDisabled())
	{
		if (m_Base->m_Goal)
			m_Base->m_Goal->Process(dTime_s);
	}

	if (m_isControlled)
	{
		#ifdef Robot_TesterCode
		// Update Mouse Controller (This is ONLY allowed to update the POV in auto pilot)
		m_mouseDriver->DriveShip();
		#endif

		//Now for the ship
		if (!AreControlsDisabled())
		{
			//factor in the auxiliary control velocities
			double AuxiliaryVelocity=0.0;
			{
				Vec2d AuxLinearAcceleration=Vec2d(0.0,0.0);
				double AuxAngularAcceleration=0.0;
				m_ship->UpdateController(AuxiliaryVelocity,AuxLinearAcceleration,AuxAngularAcceleration,m_Ship_UseHeadingSpeed,dTime_s);
				m_Ship_JoyMouse_currAccel+=AuxLinearAcceleration;
				m_Ship_JoyMouse_rotAcc_rad_s+=AuxAngularAcceleration;
			}
			m_FieldCentricDrive.TimeChange(dTime_s);

			// Normally we pass the the ship the addition of the keyboard and mouse accel
			Vec2d shipAccel = m_Ship_Keyboard_currAccel+m_Ship_JoyMouse_currAccel;

			const double ReverseScalar=m_YFlipScalar;
			// apply various input sources to current acceleration
			if (m_ship->GetAlterTrajectory())
			{
				m_ShipKeyVelocity+=(shipAccel[1]*dTime_s);
				m_ship->SetRequestedVelocity(Vec2d(shipAccel[0],(m_CruiseSpeed+AuxiliaryVelocity+m_ShipKeyVelocity)*ReverseScalar)); //this will check implicitly for which mode to use
			}
			else
				m_ship->SetCurrentLinearAcceleration(Vec2d(shipAccel[0],shipAccel[1]*ReverseScalar)); 

			
			//flush the JoyMouse current acceleration vec2 since it works on an additive nature
			m_Ship_JoyMouse_currAccel=Vec2d(0.0,0.0);
			//reset m_ShipKeyVelocity if it got its value from joystick
			if (m_Ship_Keyboard_currAccel.length2()==0.0)
				m_ShipKeyVelocity=0.0;
		
			//add all the various input types to the main rotation velocity
			double rotAcc=(m_Ship_Keyboard_rotAcc_rad_s+m_Ship_JoyMouse_rotAcc_rad_s);
			//We may have same strange undesired flicker effect if the mouse and keyboard turns are used simultaneously! So if the keyboard is used, then
			//the mouse will get ignored
			bool LockShipHeadingToOrientation=m_Ship_UseHeadingSpeed;

			if (m_Ship_Keyboard_rotAcc_rad_s!=0) 
			{
				LockShipHeadingToOrientation=true;
				rotAcc=m_Ship_Keyboard_rotAcc_rad_s;
				m_Ship_UseHeadingSpeed=true;
			}

			m_ship->SetCurrentAngularAcceleration(rotAcc,LockShipHeadingToOrientation);

			//flush the JoyMouse rotation acceleration since it works on an additive nature
			m_Ship_JoyMouse_rotAcc_rad_s=0.0;
			m_Ship_JoyMouse_currAccel=Vec2d(0.0,0.0);
			m_CruiseSpeed=0;
		}
	}
}

// enum eThrustState { TS_AfterBurner_Brake=0, TS_Brake, TS_Coast, TS_Thrust, TS_AfterBurner, TS_NotVisible };
const char* ThrustStateNames[] = {
	"Ship.AfterBurner_Brake", "Ship.Brake", "Ship.Coast", "Ship.Thrust", "Ship.AfterBurner", "Ship.NotVisible"
};

void UI_Controller::UpdateUI(double dTime_s)
{
	//No work to do if there is no ship to control
	if (!m_Base)
		return;
	// Positions:
	//[0]  +Right -Left
	//[1]  +Forward -Reverse
	//[2]  +Up -Down
	#ifdef __EnableTestKeys__
	if (m_Test1)
		m_ship->m_Physics.ApplyFractionalForce(Vec3d(0,0,m_ship->Mass),Vec3d(0,10,0),dTime_s);

	else if (m_Test2)
		m_ship->m_Physics.ApplyFractionalForce(Vec3d(0,0,-m_ship->Mass),Vec3d(0,10,0),dTime_s);
	#endif
	{
		#if 1
		Vec2d pos=m_ship->GetPos_m();
		DOUT(1,"x=%.2f y=%.2f r=%.2f",Meters2Feet(pos[0]),Meters2Feet(pos[1]),RAD_2_DEG(m_ship->GetAtt_r()));
		SmartDashboard::PutNumber("X_ft ",Meters2Feet(pos[0]));
		SmartDashboard::PutNumber("Y_ft ",Meters2Feet(pos[1]));
		SmartDashboard::PutNumber("Heading",RAD_2_DEG(m_ship->GetAtt_r()));
		#ifdef Robot_TesterCode
		Vec2d Velocity=m_ship->GetLinearVelocity_ToDisplay();
		DOUT(3,"Vel[0]=%.2f Vel[1]=%.2f Rot=%.2f mode=%s",Meters2Feet(Velocity[0]),Meters2Feet(Velocity[1]),m_ship->GetAngularVelocity_ToDisplay(),m_ship->GetAlterTrajectory()?"Sim":"Slide");
		#endif
		//These may be disabled and put from the encoder readings instead
		#if 0
		SmartDashboard::PutNumber("Velocity",Meters2Feet(Velocity[1]));
		SmartDashboard::PutNumber("Rotation Velocity",m_ship->GetAngularVelocity_ToDisplay());
		#endif
		SmartDashboard::PutBoolean("IsSlide",!m_ship->GetAlterTrajectory());
		#endif
		#if 0
		Vec2d pos=m_ship->GetPos_m();
		DOUT1("x=%f y=%f r=%f",pos[0],pos[1],RAD_2_DEG(m_ship->GetAtt_r()));
		DOUT3("Speed=%f mode=%s",m_ship->GetPhysics().GetLinearVelocity().length(),m_ship->GetAlterTrajectory()?"Sim":"Slide");
		#endif
		#if 0
		{
			GG_Framework::UI::MainWindow& mainWin = *GG_Framework::UI::MainWindow::GetMainWindow();
			Vec3 eye,center,up;
			mainWin.GetMainCamera()->GetCameraMatrix().getLookAt(eye,center,up);
			//DOUT2("%f %f %f",eye[0],eye[1],eye[2]);
			DOUT2("%f %f %f",eye[0]-pos[0],eye[1]-pos[1],eye[2]-pos[2]);
		}
		#endif
		//DebugOut_PDCB::TEXT2 = BuildString("%s", ThrustStateNames[m_ship->GetThrustState()]);
	}
}

//////////////////////////////////////////////////////////////////////////

void UI_Controller::HookUpUI(bool ui)
{
	m_isControlled = ui;
	#ifdef Robot_TesterCode
	UI::MainWindow& mainWin = *GG_Framework::UI::MainWindow::GetMainWindow();
	if (m_isControlled)
	{
		// Start with the mouse centered in the screen and turn off the cursor
		mainWin.PositionPointer(0.0f,0.0f);
		mainWin.UseCursor(false);

		// Provide the new camera manipulator
		mainWin.GetKeyboard_Mouse().SetControlledEventMap(m_ship->GetEventMap());
		mainWin.GetJoystick().SetControlledEventMap(m_ship->GetEventMap());
	}
	#endif

	if (m_ship)
	{
		// Set Auto Pilot to true if there is control
		SetAutoPilot(!m_isControlled || !m_Base->GetCanUserPilot());
	}
}
//////////////////////////////////////////////////////////////////////////

void UI_Controller::OnSpawn(bool on)
{
	// Set Auto Pilot to true if there is control.
	// This is commented out for now so we can leave a ship in auto-pilot and it will return to auto-pilot after spawn
	// SetAutoPilot(!m_isControlled || !m_Base->GetCanUserPilot());
}




