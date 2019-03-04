#pragma once

#ifdef Robot_TesterCode
extern COMMON_API bool g_UseMouse;

class Mouse_ShipDriver
{
public:
	Mouse_ShipDriver(Ship_2D& ship,UI_Controller *parent, unsigned avgFrames);
	void OnMouseMove(float mx, float my);
	void DriveShip();

private:
	// Use this handler to tie events to this manipulator
	IEvent::HandlerList ehl;
	void OnMouseRoll(bool onoff){m_mouseRoll = onoff;}

	Ship_2D& m_ship;
	UI_Controller * const m_ParentUI_Controller;
	osg::Vec2f m_lastMousePos;
	osg::Vec2f* m_mousePosHist;
	unsigned m_avgFrames;
	unsigned m_currFrame;
	bool m_mouseRoll;
};
#endif

class COMMON_API UI_Controller
{
	public:
		enum Directions
		{
			Dir_None = 0,
			//Reversed from game since the 2D world uses the topview axis
			Dir_Left = -1,
			Dir_Right = 1,
			Dir_90Left,
			Dir_90Right,
			Dir_180
		};

		#ifdef Robot_TesterCode
		UI_Controller(AI_Base_Controller *base_controller=NULL,bool AddJoystickDefaults=true);
		#else
		UI_Controller(Framework::UI::JoyStick_Binder *joy,AI_Base_Controller *base_controller=NULL);
		#endif

		virtual ~UI_Controller();

		///This is the most important method, as we must have a ship to control
		void Set_AI_Base_Controller(AI_Base_Controller *controller);
		void TryFireMainWeapon(bool on)
		{
			if (AreControlsDisabled() && on) return;
			//m_ship->GetEventMap()->EventOnOff_Map["Ship.TryFireMainWeapon"].Fire(on);
		}

		void AfterBurner_Thrust(bool on){if (AreControlsDisabled() && on) return; Ship_AfterBurner_Thrust(on);}
		void Thrust(bool on){if (AreControlsDisabled() && on) return; Ship_Thrust(on);}
		void Slider_Accel(double Intensity);
		void Brake(bool on){if (AreControlsDisabled() && on) return; Ship_Brake(on);}
		void Stop() {if (AreControlsDisabled()) return; m_ShipKeyVelocity=0.0;m_ship->Stop();}
		void MatchSpeed(double speed) {if (AreControlsDisabled()) return; m_ship->SetRequestedVelocity(speed);}
		void Quadrant_SetCurrentSpeed(double NormalizedVelocity);

		void Turn_R(bool on){if (AreControlsDisabled() && on) return; Ship_Turn(on?Dir_Right:Dir_None);}
		void Turn_L(bool on){if (AreControlsDisabled() && on) return; Ship_Turn(on?Dir_Left:Dir_None);}
		void Turn_90R() {if (AreControlsDisabled()) return; Ship_Turn(Dir_90Right);}
		void Turn_90L() {if (AreControlsDisabled()) return; Ship_Turn(Dir_90Left);}
		void Turn_180() {if (AreControlsDisabled()) return; Ship_Turn(Dir_180);}
		void Turn_180_Hold(bool on);
		void FlipY();
		void FlipY_Hold(bool on) {if (on) FlipY();}
		/// \param Absolute you can use method to set absolute positions as well
		void Turn_RelativeOffset(double value,bool Absolute=false);

		virtual void ResetPos();
		void UserResetPos();
		void SlideHold(bool holdslide) {if (AreControlsDisabled()) return; m_ship->SetSimFlightMode(!holdslide);}
		void ToggleSlide() {if (AreControlsDisabled()) return; m_SlideButtonToggle=!m_ship->GetAlterTrajectory(),m_ship->SetSimFlightMode(m_SlideButtonToggle);}
		void ToggleAutoLevel();
		void StrafeLeft(bool on) {if (AreControlsDisabled() && on) return; Ship_StrafeLeft(on);}
		void StrafeLeft(double dir) {if (!AreControlsDisabled()) Ship_StrafeLeft(dir); }
		void StrafeRight(bool on) {if (AreControlsDisabled() && on) return; Ship_StrafeRight(on);}
		void StrafeRight(double dir) {if (!AreControlsDisabled()) Ship_StrafeRight(dir); }

		void TryToggleAutoPilot();

		// This may not always happen.  Some ships can ONLY be in auto pilot
		bool SetAutoPilot(bool autoPilot);
		bool GetAutoPilot() const {return m_autoPilot;}

		void OnSpawn(bool on);

		void UseMouse();
		void Test1(bool on);
		void Test2(bool on);

		//osg::Geometry* MakeVelLine(osg::Vec2 vel);
		void HookUpUI(bool ui);
		bool GetControlled(){return m_isControlled;}

		///This is called from the ship's time change (first before the ship does its physics)
		virtual void UpdateController(double dTime_s);
		void UpdateUI(double dTime_s);

		virtual void CancelAllControls();

		//returns NULL if no error
		struct Controller_Element_Properties
		{
			std::string Event;
			enum ElementType
			{
				eJoystickAnalog,
				eJoystickButton
			} Type;
			union ElementTypeSpecific
			{
				struct AnalogSpecifics_rw
				{
					UI::JoyStick_Binder::JoyAxis_enum JoyAxis;
					bool IsFlipped;
					double Multiplier;
					double FilterRange;
					double CurveIntensity;
				} Analog;
				struct ButtonSpecifics_rw
				{
					size_t WhichButton;
					bool useOnOff;
					bool dbl_click;
				} Button;
			} Specifics;
		};
		//Return if element was successfully created (be sure to check as some may not be present)
		static const char *ExtractControllerElementProperties(Controller_Element_Properties &Element,const char *Eventname,Scripting::Script& script);
		UI::JoyStick_Binder &GetJoyStickBinder();
		UI::KeyboardMouse_CB *GetKeyboardBinder();  //returns null if not supported
	protected:
		#ifdef Robot_TesterCode
		friend Mouse_ShipDriver;
		#endif

		void BlackoutHandler(double bl);

		///All non-right button mouse movements will come here to be dispatched to the proper place
		void Mouse_Turn(double dir);

		/// \param dir When UseHeadingSpeed=true this is a velocity scaler applied to the scripted turning speed.
		/// When UseHeadingSpeed=false the dir parameter becomes the actual velocity in radians per second.
		/// \param UseHeadingSpeed Typically the Joystick and Keyboard will set UseHeadingSpeed to true, and mouse will be false
		/// when this is true it also implicitly applies the ships torque restraints to the intended speed.
		/// \todo implement these to work with intended orientation quaternion.
		void Ship_Turn(double dir,bool UseHeadingSpeed=true);

		void JoyStick_Ship_Turn(double dir);

		void Ship_Turn(Directions dir);
		void Ship_Turn90_POV (double value);

		void Ship_AfterBurner_Thrust(bool on);
		void Ship_Thrust(bool on);
		void Ship_Brake(bool on);

		void Ship_Thrust(double Intensity);
		void Ship_Brake(double Intensity);

		void Joystick_SetCurrentSpeed(double Speed);
		void Joystick_SetCurrentSpeed_2(double Speed);

		void Ship_StrafeLeft(bool on)	{		m_Ship_Keyboard_currAccel[0]= on? -m_ship->GetStrafeSpeed() : 0.0;	}
		void Ship_StrafeRight(bool on)	{		m_Ship_Keyboard_currAccel[0]= on? m_ship->GetStrafeSpeed() : 0.0;	}

		void Ship_StrafeLeft(double Intensity);
		void Ship_StrafeRight(double Intensity);

		IEvent::HandlerList ehl;
	private:
		void Flush_AI_BaseResources(); //i.e. chase plane and mouse driver
		void Init_AutoPilotControls();
		AI_Base_Controller *m_Base;
		Ship_2D *m_ship; //there is an overwhelming use of the ship so we'll cache a pointer of it here
		
		#ifdef Robot_TesterCode
		Mouse_ShipDriver *m_mouseDriver;
		#else
		UI::JoyStick_Binder *m_JoyStick_Binder;
		UI::KeyboardMouse_CB m_Keyboard;
		#endif

		double m_LastSliderTime[2]; //Keep track of the slider to help it stay smooth;
		bool m_isControlled;

		///This is used exclusively for keyboard turn methods
		double m_Ship_Keyboard_rotAcc_rad_s;
		///This one is used exclusively for the Joystick and Mouse turn methods
		double m_Ship_JoyMouse_rotAcc_rad_s;
		Vec2D m_Ship_Keyboard_currAccel,m_Ship_JoyMouse_currAccel;
		double m_ShipKeyVelocity;

		//I have to monitor when it is down then up
		bool m_SlideButtonToggle;
		bool m_FireButton;
		double m_CruiseSpeed; ///< This is used with the Joystick control to only apply speed changes when a change occurs
		double m_YFlipScalar;  ///< Used to dynamically flip the orientation of the Y controls (Always 1.0 or -1.0)

		// Are we flying in auto-pilot?
		bool m_autoPilot;
		bool m_enableAutoLevelWhenPiloting;

		// Are we disabling UI controls?
		bool AreControlsDisabled();

		bool m_Ship_UseHeadingSpeed;
		bool m_Test1,m_Test2; //Testing
		bool m_IsBeingDestroyed; //Keep track of when destructor is called
		bool m_POVSetValve;

		class FieldCentricDrive
		{
		private:
			UI_Controller * const m_pParent;
			double m_PosX,m_PosY;
			double m_HeadingLock;
			double m_XAxisEnableThreshold;
			bool m_FieldCentricDrive_Mode;  //is in this mode if true
		protected:
			void UpdatePosY(double Y) {m_PosY=Y;}
			void UpdatePosX(double X) {m_PosX=X;}
			void FieldCentricDrive_Mode_Enable();
			void FieldCentricDrive_Mode_Enable(double Value);
		public:
			FieldCentricDrive(UI_Controller *pParent);
			void TimeChange(double dTime_s);
			void BindAdditionalEventControls(bool Bind,Base::EventMap *em,IEvent::HandlerList &ehl);
		} m_FieldCentricDrive;
};
