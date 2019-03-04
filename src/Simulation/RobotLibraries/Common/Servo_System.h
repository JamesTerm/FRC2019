#pragma once

struct Servo_Props
{
	double ServoScalar,ServoOffset;  //used to translate desired angle range and offset to actual range and offset
	double PrecisionTolerance;  //Used to avoid oscillation during set point adjustments

	size_t Feedback_DiplayRow;  //Choose a row for display -1 for none (Only active if __DebugLUA__ is defined)
	//bool PID_Console_Dump;  //This will dump the console PID info (Only active if __DebugLUA__ is defined)
};

///This is the next layer of the linear Ship_1D that converts velocity into displacement angles, on a system that has servos
///This is used for fixed point position setting... like a turret or arm that works with servos
class COMMON_API Servo_Position_Control : public Ship_1D
{
	private:
		#ifndef _Win32
		typedef Ship_1D __super;
		#endif 
		//Copy these lines to the subclass that binds the events
		//events are a bit picky on what to subscribe so we'll just wrap from here
		//void SetRequestedVelocity_FromNormalized(double Velocity) {__super::SetRequestedVelocity_FromNormalized(Velocity);}

		/// \param DisableFeedback this allows ability to bypass feedback
		Servo_Control_Interface * const m_RobotControl;
		const size_t m_InstanceIndex;
		Servo_Props m_Servo_Props;

		double m_LastPosition;  //used for calibration
		double m_MatchVelocity;
		double m_ErrorOffset;
		double m_LastTime; //used for calibration
		double m_MaxSpeedReference; //used for calibration
		double m_PreviousVelocity; //used to compute acceleration
	public:
		Servo_Position_Control(const char EntityName[],Servo_Control_Interface *robot_control,size_t InstanceIndex=0);
		IEvent::HandlerList ehl;
		//The parent needs to call initialize
		virtual void Initialize(Base::EventMap& em,const Entity1D_Properties *props=NULL);
		virtual void ResetPos();
		const Servo_Props &GetServo_Properties() const {return m_Servo_Props;}
		//This is optionally used to lock to another ship (e.g. drive using Servo system)
		void SetMatchVelocity(double MatchVel) {m_MatchVelocity=MatchVel;}
	protected:
		//Intercept the time change to obtain current height as well as sending out the desired velocity
		virtual void TimeChange(double dTime_s);
		virtual double GetMatchVelocity() const {return m_MatchVelocity;}
};

class COMMON_API Servo_Properties : public Ship_1D_Properties
{
	public:
		void Init();
		Servo_Properties(const char EntityName[], double Mass,double Dimension,
			double MAX_SPEED,double ACCEL,double BRAKE,double MaxAccelForward, double MaxAccelReverse,	
			Ship_Type ShipType=Ship_1D_Props::eDefault, bool UsingRange=false, double MinRange=0.0, double MaxRange=0.0,
			bool IsAngular=false) : Ship_1D_Properties(EntityName,Mass,Dimension,MAX_SPEED,ACCEL,BRAKE,MaxAccelForward,
			MaxAccelReverse,ShipType,UsingRange,MinRange,MaxRange,IsAngular) {Init();}

		Servo_Properties() {Init();}
		virtual void LoadFromScript(Scripting::Script& script, bool NoDefaults=false);
		const Servo_Props &GetServoProps() const {return m_ServoProps;}
		//Get and Set the properties
		Servo_Props &ServoProps() {return m_ServoProps;}
	protected:
		Servo_Props m_ServoProps;
	private:
		#ifndef _Win32
		typedef Ship_1D_Properties __super;
		#endif
};
