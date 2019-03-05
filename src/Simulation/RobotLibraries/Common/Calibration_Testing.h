#pragma once

class COMMON_API Potentiometer_Tester : public Ship_1D
{
	public:
		Potentiometer_Tester();
		void UpdatePotentiometerVoltage(double Voltage);
		virtual double GetPotentiometerCurrentPosition();
		//This is broken up so that the real interface does not have to pass time
		void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
		void TimeChange();
		void SetBypass(bool bypass) {m_Bypass=bypass;}
	protected:
	private:
		double m_Time_s;
		Ship_1D_Properties m_PotentiometerProps;
		Framework::Base::EventMap m_DummyMap;
		bool m_Bypass;  //used for stress test
};

class COMMON_API Potentiometer_Tester2 : public Ship_1D
{
	public:
		Potentiometer_Tester2();
		virtual void Initialize(const Ship_1D_Properties *props=NULL);

		void UpdatePotentiometerVoltage(double Voltage);
		virtual double GetPotentiometerCurrentPosition();
		//This is broken up so that the real interface does not have to pass time
		void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
		void TimeChange();
		void SetSimulateOpposingForce(bool Simulate) {m_SimulateOpposingForce=Simulate;}
	protected:
	private:
		void SimulateOpposingForce(double Voltage);
		double m_Time_s;
		Ship_1D_Properties m_PotentiometerProps;
		Framework::Base::EventMap m_DummyMap;
		bool m_SimulateOpposingForce;  //used for stress test
		#ifndef _Win32
		typedef Ship_1D __super;
		#endif

};

//This simulates the 10 turn poteniometer range of 5 - 4059
class Potentiometer_AM2619_3590 : public Potentiometer_Tester2
{
private:
	double m_Scale=1.0;
	double m_Offset=0.0;
public:
	//The native units of the rotary system need to be normalized where 0 is absolute beginning, and 1 is the absolute end, where it accounts for the padding
	//Use scale and offset to factor it as such
	void SetNormalizedProperties(double scale, double offset) { m_Scale = scale, m_Offset = offset; }
	//This will provide the actual number needed
	__inline double GetActualPotPosition()
	{	const double native = GetPotentiometerCurrentPosition();
		const double norm = native * m_Scale + m_Offset;
		return norm * 4054.0 + 5.0;
	}
};

class COMMON_API Encoder_Simulator : public Ship_1D
{
	public:
		Encoder_Simulator(const char *EntityName="EncSimulator");
		virtual void Initialize(const Ship_1D_Properties *props=NULL);

		void UpdateEncoderVoltage(double Voltage);
		virtual double GetEncoderVelocity();
		//This is broken up so that the real interface does not have to pass time
		void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
		void TimeChange();
		void SetReverseDirection(bool reverseDirection);  //emulates functionality of the encoder (needed because kids put them in differently)
		void SetEncoderScalar(double value) {m_EncoderScalar=value;}  //This helps to simulate differences between sides
		void SetFriction(double StaticFriction,double KineticFriction) {}
	protected:
	private:
		double m_Time_s;
		Ship_1D_Properties m_EncoderProps;
		Framework::Base::EventMap m_DummyMap;
		LatencyFilter m_Latency;
		double m_EncoderScalar; //used to implement reverse
		bool m_GetEncoderFirstCall;  //allows GetEncoderVelocity to know when a new set of calls occur within a time slice
		#ifndef _Win32
		typedef Ship_1D __super;
		#endif
};

//TorqueAccelerationDampener - formerly known as ForceAppliedOnWheelRadius has become the solution to account for empirical testing of torque acceleration 
//measured in the motor, unfortunately I haven't yet found a way to compute for this, but this is quite effective as its factoring happens in the best
//place (same place as it was before)
struct EncoderSimulation_Props
{
	double Wheel_Mass;  //This is a total mass of all the wheels and gears for one side
	double COF_Efficiency;
	double GearReduction;  //In reciprocal form of spread sheet   driving gear / driven gear
	double TorqueAccelerationDampener; //ratio 1.0 no change
	double DriveWheelRadius; //in meters
	double NoMotors;  //Used to get total torque
	double PayloadMass;  //The robot weight in kg
	double SpeedLossConstant;
	double DriveTrainEfficiency;

	struct Motor_Specs
	{
		double FreeSpeed_RPM;
		double Stall_Torque_NM;
		double Stall_Current_Amp;
		double Free_Current_Amp;
	} motor;
};

//This is used in calibration testing to simulate encoder readings
class COMMON_API EncoderSimulation_Properties
{
	public:
		EncoderSimulation_Properties();
		virtual void LoadFromScript(Framework::Scripting::Script& script);
		const EncoderSimulation_Props &GetEncoderSimulationProps() const {return m_EncoderSimulation_Props;}
		//Get and Set the properties
		EncoderSimulation_Props &EncoderSimulationProps() {return m_EncoderSimulation_Props;}

	protected:
		EncoderSimulation_Props m_EncoderSimulation_Props;
};


class COMMON_API Drive_Train_Characteristics
{
	public:
		Drive_Train_Characteristics();
		void UpdateProps(const EncoderSimulation_Props &props) {m_Props=props;}

		__inline double GetAmp_To_Torque_nm(double Amps) const;
		__inline double INV_GetVel_To_Torque_nm(double Vel_rps) const;  //depreciated
		__inline double GetVel_To_Torque_nm(double motor_Vel_rps) const;
		__inline double GetTorque_To_Vel_nm_V1(double motor_Vel_rps) const;  //depreciated

		__inline double GetWheelTorque(double Torque) const;
		__inline double INV_GetWheelTorque(double Torque) const; //depreciated
		__inline double GetWheelStallTorque() const {return m_Props.motor.Stall_Torque_NM / m_Props.GearReduction * m_Props.DriveTrainEfficiency;}

		__inline double GetTorqueAtWheel(double Torque) const;
		__inline double GetWheelRPS(double LinearVelocity) const;
		__inline double GetLinearVelocity(double wheel_RPS) const;
		__inline double GetLinearVelocity_WheelAngular(double wheel_AngularVelocity) const;
		__inline double GetMotorRPS(double LinearVelocity) const;
		__inline double GetWheelRPS_Angular(double wheel_AngularVelocity) const;
		__inline double GetWheelAngular_RPS(double wheel_RPS) const;
		__inline double GetWheelAngular_LinearVelocity(double LinearVelocity) const;  //accurate as long as there is no skid
		__inline double GetMotorRPS_Angular(double wheel_AngularVelocity) const;
		__inline double INV_GetMotorRPS_Angular(double wheel_AngularVelocity) const; //depreciated
		__inline double GetTorqueFromLinearVelocity(double LinearVelocity) const;
		__inline double GetWheelTorqueFromVoltage(double Voltage) const;
		__inline double GetTorqueFromVoltage_V1(double Voltage) const; //depreciated
		__inline double GetTorqueFromVoltage(double Voltage) const;
		__inline double INV_GetTorqueFromVelocity(double wheel_AngularVelocity) const;  //depreciated
		__inline double GetTorqueFromVelocity(double wheel_AngularVelocity) const;

		__inline double GetMaxTraction() const {return m_Props.PayloadMass*m_Props.COF_Efficiency;}
		__inline double GetMaxDriveForce() const {return GetWheelStallTorque()/m_Props.DriveWheelRadius*2.0;}
		__inline double GetCurrentDriveForce(double WheelTorque) const {return WheelTorque/m_Props.DriveWheelRadius*2.0;}
		__inline double GetMaxPushingForce() const;

		const EncoderSimulation_Props &GetDriveTrainProps() const {return m_Props;}
		void SetGearReduction(double NewGearing) {m_Props.GearReduction=NewGearing;}
	private:
		EncoderSimulation_Props m_Props;
};

class COMMON_API Encoder_Simulator2
{
	public:
		Encoder_Simulator2(const char *EntityName="EncSimulator");
		virtual void Initialize(const Ship_1D_Properties *props=NULL);

		virtual void UpdateEncoderVoltage(double Voltage);
		virtual double GetEncoderVelocity() const;
		double GetDistance() const;

		//This is broken up so that the real interface does not have to pass time
		void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
		virtual void TimeChange();
		void SetReverseDirection(bool reverseDirection);  //emulates functionality of the encoder (needed because kids put them in differently)
		void SetEncoderScalar(double value) {m_EncoderScalar=value;}  //This helps to simulate differences between sides
		void SetFriction(double StaticFriction,double KineticFriction) {m_Physics.SetFriction(StaticFriction,KineticFriction);}
		virtual void ResetPos();
		void SetGearReduction(double NewGearing) {m_DriveTrain.SetGearReduction(NewGearing);}
	protected:
		//The wheel physics records the velocity of the wheel in radians
		PhysicsEntity_1D m_Physics;
		Drive_Train_Characteristics m_DriveTrain;
		double m_Time_s;

		double m_Position;  //also keep track of position to simulate distance use case (i.e. used as a potentiometer)
		double m_EncoderScalar; //used for position updates
		double m_ReverseMultiply; //used to implement set reverse direction
};

class COMMON_API Encoder_Simulator3 : public Encoder_Simulator2
{
public:
	//The payload mass is shared among multiple instances per motor, so each instance will need to identify the kind it is
	//multiple instance can read, but only one write per side.  Use ignore and the payload computations will be bypassed
	enum EncoderKind
	{
		eIgnorePayload,  //an easy way to make it on the bench
		eReadOnlyLeft,
		eReadOnlyRight,
		eRW_Left,
		eRW_Right
	};
	Encoder_Simulator3(const char *EntityName="EncSimulator");
	//has to be late binding for arrays
	void SetEncoderKind(EncoderKind kind);
	virtual void Initialize(const Ship_1D_Properties *props=NULL);
	virtual void UpdateEncoderVoltage(double Voltage);
	virtual double GetEncoderVelocity() const;
	virtual void TimeChange();
	virtual void ResetPos();
protected:
	double m_Voltage; //cache voltage for speed loss assistance
	//We are pulling a heavy mass this will present more load on the wheel, we can simulate a bench test vs. an actual run by factoring this in
	static PhysicsEntity_1D s_PayloadPhysics_Left;
	static PhysicsEntity_1D s_PayloadPhysics_Right;
	//Cache last state to determine if we are accelerating or decelerating
	static double s_PreviousPayloadVelocity_Left;
	static double s_PreviousPayloadVelocity_Right;  
	double m_PreviousWheelVelocity;
	size_t m_EncoderKind;
private:
		#ifndef _Win32
		typedef Encoder_Simulator2 __super;
		#endif

};

class COMMON_API Potentiometer_Tester3 : public Encoder_Simulator2
{
public:
	Potentiometer_Tester3(const char *EntityName="PotSimulator3");
	virtual void Initialize(const Ship_1D_Properties *props=NULL);

	void UpdatePotentiometerVoltage(double Voltage);
	virtual double GetPotentiometerCurrentPosition();
	virtual void TimeChange();
	virtual void ResetPos();
protected:

	double m_InvEncoderToRS_Ratio;
private:
	std::queue<double> m_Slack;  //when going down this will grow to x frames, and going up with shrink... when not moving it will shrink to nothing
	double m_SlackedValue; // ensure the pop of the slack only happens in the time change
	#ifndef _Win32
	typedef Encoder_Simulator2 __super;
	#endif
};

class COMMON_API Encoder_Tester
{
	public:
		Encoder_Tester();
		virtual void Initialize(const Ship_1D_Properties *props=NULL);
		virtual void GetLeftRightVelocity(double &LeftVelocity,double &RightVelocity);
		virtual void UpdateLeftRightVoltage(double LeftVoltage,double RightVoltage);
		void SetTimeDelta(double dTime_s);
		void TimeChange();

		void SetLeftRightReverseDirectionEncoder(bool Left_reverseDirection,bool Right_reverseDirection)
		{
			m_LeftEncoder.SetReverseDirection(Left_reverseDirection),m_RightEncoder.SetReverseDirection(Right_reverseDirection);
		}
		void SetLeftRightScalar(double LeftScalar,double RightScalar)
		{
			m_LeftEncoder.SetEncoderScalar(LeftScalar),m_RightEncoder.SetEncoderScalar(RightScalar);
		}
	private:
		#if 0
		//This is still good for a lesser stress (keeping the latency disabled)
		Encoder_Simulator m_LeftEncoder;
		Encoder_Simulator m_RightEncoder;
		#else
		Encoder_Simulator2 m_LeftEncoder;
		Encoder_Simulator2 m_RightEncoder;
		#endif
};