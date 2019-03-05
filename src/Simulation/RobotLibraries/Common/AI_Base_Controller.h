#pragma once

//An aggregated type of control for robots that wish to use this kind of steering
class COMMON_API Tank_Steering
{
	private:
		Ship_2D * const m_pParent;
		double m_LeftVelocity, m_RightVelocity;  //for tank steering
		double m_LeftXAxis,m_RightXAxis;
		double m_StraightDeadZone_Tolerance;  //used to help controls drive straight
		bool m_90DegreeTurnValve;
		bool m_AreControlsDisabled;
	public:
		Tank_Steering(Ship_2D *parent);

		void SetAreControlsDisabled(bool AreControlsDisabled) {m_AreControlsDisabled=AreControlsDisabled;}
		//This is the ui controllers time change callback update... client code must handle initializing as this will only write to those
		//that need to be written to
		void UpdateController(double &AuxVelocity,Vec2D &LinearAcceleration,double &AngularAcceleration,const Ship_2D &ship,bool &LockShipHeadingToOrientation,double dTime_s);
		void BindAdditionalEventControls(bool Bind,Base::EventMap *em,IEvent::HandlerList &ehl);

		//This method is depreciated and called internally within BindAdditionalEventControls
		//range 0-1 the higher this is the lower turning precision, but easier to drive straight
		//void SetStraightDeadZone_Tolerance(double Tolerance) {m_StraightDeadZone_Tolerance=Tolerance;}
	protected:
		void Joystick_SetLeftVelocity(double Velocity);
		void Joystick_SetRightVelocity(double Velocity);
		void Joystick_SetLeft_XAxis(double Value);
		void Joystick_SetRight_XAxis(double Value);
};

class COMMON_API AI_Base_Controller
{
	public:
		AI_Base_Controller(Ship_2D &ship);
		virtual ~AI_Base_Controller() {}

		///This is the single update point to all controlling of the ship.  The base class contains no goal arbitration, but does implement
		///Whatever goal is passed into it if the UI controller is off line
		virtual void UpdateController(double dTime_s);

		//This is mostly for pass-thru since the timing of this is in alignment during a ship att-pos update
		void UpdateUI(double dTime_s);

		/// I put the word try, as there may be some extra logic to determine if it has permission
		/// This is a bit different than viewing an AI with no controls, where it simply does not
		/// Allow a connection
		/// \return true if it was allowed to bind
		virtual bool Try_SetUIController(UI_Controller *controller);
		virtual void ResetPos() {}

		bool HasAutoPilotRoute() {return true;}
		bool GetCanUserPilot() {return true;}

		void SetShipVelocity(double velocity_mps) {m_ship.SetRequestedVelocity(velocity_mps);}

		/// \param TrajectoryPoint- This is the point that your nose of your ship will orient to from its current position (usually the same as PositionPoint)
		/// \param PositionPoint- This is the point where your ship will be to position to (Usually the same as TrajectoryPoint)
		/// \power- The scaled value multiplied to the ships max speed.  If > 1 it can be interpreted as explicit meters per second speed
		/// \matchVel- You can make it so the velocity of the ship when it reaches the point.  
		/// Use NULL when flying through way-points
		/// Use (0,0) if you want to come to a stop, like at the end of a way-point series
		/// Otherwise, use the velocity of the ship you are targeting or following to keep up with it
		void DriveToLocation(Vec2D TrajectoryPoint,Vec2D PositionPoint, double power, double dTime_s,Vec2D* matchVel,bool LockOrientation=false);
		void SetIntendedOrientation(double IntendedOrientation) {m_ship.SetIntendedOrientation(IntendedOrientation);}
		
		Ship_2D &GetShip() {return m_ship;}
		const UI_Controller *GetUIController() const {return m_UI_Controller;}
		//I want it to be clear when we intend to write
		UI_Controller *GetUIController_RW() {return m_UI_Controller;}
	protected:
		friend class Ship_Tester;
		Goal *m_Goal; //Dynamically set a goal for this controller
	private:
		//TODO determine way to properly introduce UI_Controls here	
		Ship_2D &m_ship;

		friend class UI_Controller;
		UI_Controller *m_UI_Controller;
};

//This will explicitly rotate the ship to a particular heading.  It may be moving or still.
class COMMON_API Goal_Ship_RotateToPosition : public AtomicGoal
{
	public:
		Goal_Ship_RotateToPosition(AI_Base_Controller *controller,double Heading);
		~Goal_Ship_RotateToPosition();
		virtual void Activate();
		virtual Goal_Status Process(double dTime_s);
		virtual void Terminate() {m_Terminate=true;}
	protected:
		AI_Base_Controller * const m_Controller;
		double m_Heading;
		Ship_2D &m_ship;
	private:
		bool m_Terminate;
};


//This is like Goal_Ship_RotateToPosition except it will have a relative heading added to its current heading
class COMMON_API Goal_Ship_RotateToRelativePosition : public Goal_Ship_RotateToPosition
{
public:
	Goal_Ship_RotateToRelativePosition(AI_Base_Controller *controller,double Heading) : 
	  Goal_Ship_RotateToPosition(controller,Heading) {}
	//Note: It is important for client code not to activate this... let process activate it... so that it sets the heading at the correct time and current heading
	virtual void Activate();
private:
#ifndef _Win32
	typedef Goal_Ship_RotateToPosition __super;
#endif
};

//TODO get these functions re-factored
//Update Reaction->FlyWayPoints->UpdateIndendedLocation
//FlyToNextLocation->DriveToLocation

struct COMMON_API WayPoint
{
	WayPoint() : Power(0.0), Position(0,0),TurnSpeedScaler(1.0) {}
	double Power;
	Vec2D Position;
	double TurnSpeedScaler;  //This will have a default value if not in script
};

//This is similar to Traverse_Edge in book (not to be confused with its MoveToPosition)
class COMMON_API Goal_Ship_MoveToPosition : public AtomicGoal
{
	public:
		/// \param double safestop_tolerance used to set safe stop tolerance, default is a little over an inch
		Goal_Ship_MoveToPosition(AI_Base_Controller *controller,const WayPoint &waypoint,bool UseSafeStop=true,
				bool LockOrientation=false,double safestop_tolerance=0.03);
		~Goal_Ship_MoveToPosition();
		//optionally set the trajectory point... it is the same as the waypoint by default
		void SetTrajectoryPoint(const Vec2D &TrajectoryPoint);
		virtual void Activate();
		virtual Goal_Status Process(double dTime_s);
		virtual void Terminate();

	protected:
		//Similar to FlyWayPoints, except it only checks for the tolerance
		bool HitWayPoint();  

		WayPoint m_Point;
		Vec2D m_TrajectoryPoint;
		AI_Base_Controller * const m_Controller;
		Ship_2D &m_ship;
		double m_SafeStopTolerance;
		bool m_Terminate;
		bool m_UseSafeStop;
		bool m_LockOrientation;
};

//This is like Goal_Ship_MoveToPosition except it will set the waypoint relative to its current position and orientation
//This will also set the trajectory point x distance (1 meter default) beyond the the point to help assist in orientation
class COMMON_API Goal_Ship_MoveToRelativePosition : public Goal_Ship_MoveToPosition
{
public:
	Goal_Ship_MoveToRelativePosition(AI_Base_Controller *controller,const WayPoint &waypoint,bool UseSafeStop=true,
		bool LockOrientation=false,double safestop_tolerance=0.03) : Goal_Ship_MoveToPosition(controller,waypoint,UseSafeStop,LockOrientation,safestop_tolerance) {}
	//Note: It is important for client code not to activate this... let process activate it... so that it sets the point at the correct time and current position
	virtual void Activate();
private:
	#ifndef _Win32
	typedef Goal_Ship_MoveToPosition __super;
	#endif
};

class COMMON_API Goal_Ship_FollowPath : public CompositeGoal
{
	public:
		Goal_Ship_FollowPath(AI_Base_Controller *controller,std::list<WayPoint> path,bool LoopMode=false,bool UseSafeStop=false);
		virtual void Activate();
		virtual Goal_Status Process(double dTime_s);
		virtual void Terminate();
	private:
		AI_Base_Controller * const m_Controller;
		std::list<WayPoint> m_Path,m_PathCopy;
		bool m_LoopMode,m_UseSafeStop;
};

class COMMON_API Goal_Ship_FollowShip : public AtomicGoal
{
	public:
		/// \param Trajectory_ForwardOffset This control where the orientation of the following ship will look.  This can vary depending on the size
		/// of the ship.  This should be virtually 0 if the ship has no strafe
		Goal_Ship_FollowShip(AI_Base_Controller *controller,const Ship_2D &Followship,const Vec2D &RelPosition,double Trajectory_ForwardOffset=100.0);
		~Goal_Ship_FollowShip();
		virtual void Activate();
		virtual Goal_Status Process(double dTime_s);
		virtual void Terminate();
		//Allow client to change its relative position dynamically
		void SetRelPosition(const Vec2D &RelPosition);
	private:
		AI_Base_Controller * const m_Controller;
		Vec2D m_RelPosition,m_TrajectoryPosition;
		double m_TrajectoryPosition_ForwardOffset; //The amount forward to extend the trajectory point
		const Ship_2D &m_Followship;
		Ship_2D &m_ship;
		bool m_Terminate;
};

class COMMON_API Goal_Wait : public AtomicGoal
{
	public:
		Goal_Wait(double seconds);
		virtual void Activate();
		virtual Goal_Status Process(double dTime_s);
		virtual void Terminate();
	private:
		double m_TimeAccrued;
		double m_TimeToWait;
};

//This goal simply will fire an event when all goals are complete
class COMMON_API Goal_NotifyWhenComplete : public CompositeGoal
{
	private:
		#ifndef _Win32
		typedef CompositeGoal __super;
		#endif
		std::string m_EventName,m_FailedEventName;  //name to fire when complete
		Base::EventMap &m_EventMap;
	public:
		Goal_NotifyWhenComplete(Base::EventMap &em,const char *EventName,const char *FailedEventName=NULL);
		//give public access for client to populate goals
		virtual void AddSubgoal(Goal *g) {__super::AddSubgoal(g);}
		//client activates manually when goals are added
		virtual void Activate();
		virtual Goal_Status Process(double dTime_s);
		virtual void Terminate();
};
