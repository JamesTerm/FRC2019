#pragma once
#include <functional>
#include <memory>
#ifndef _Win32
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#endif

namespace frc
{
//This parses out the LUA into two table for each control element... its population properties and LUT
class COMMON_API Control_Assignment_Properties
{
	public:
		virtual ~Control_Assignment_Properties() {}
		struct Control_Element_1C
		{
			std::string type; //Used to distinguish derived kinds of the same group (e.g. Victor, VictorSP)
			std::string name;
			size_t Channel;
			size_t Module;
		};
		struct Control_Element_2C
		{
			std::string name;
			size_t ForwardChannel,ReverseChannel;
			size_t Module;
		};
		typedef std::vector<Control_Element_1C> Controls_1C;
		typedef std::vector<Control_Element_2C> Controls_2C;

		virtual void LoadFromScript(Scripting::Script& script);

		const Controls_1C &GetPWMSpeedControllers() const {return m_PWMSpeedControllers;}
		const Controls_1C &GetVictorSPXs() const { return m_VictorSPXs; }
		const Controls_1C &GetServos() const {return m_Servos;}
		const Controls_1C &GetRelays() const {return m_Relays;}
		const Controls_1C &GetDigitalInputs() const {return m_Digital_Inputs;}
		const Controls_1C &GetAnalogInputs() const {return m_Analog_Inputs;}
		const Controls_2C &GetDoubleSolenoids() const {return m_Double_Solenoids;}
		const Controls_2C &GetEncoders() const {return m_Encoders;}
		size_t GetCompressorRelay() {return m_Compressor_Relay;}
		size_t GetCompressorLimit() {return m_Compressor_Limit;}
	private:
		Controls_1C m_PWMSpeedControllers,m_VictorSPXs,m_Servos,m_Relays,m_Digital_Inputs,m_Analog_Inputs;
		Controls_2C m_Double_Solenoids,m_Encoders;
		size_t m_Compressor_Relay=-1,m_Compressor_Limit=-1;
};


//Add simulated WPILib control elements here... these are to reflect the latest compatible interface with the WPI libraries
#ifdef _Win32
typedef unsigned     int uint32_t;
typedef unsigned    char uint8_t;
typedef unsigned	 int UINT32;
typedef              int int32_t;
typedef			   short int16_t;

class Control_1C_Element_UI
{
public:
	Control_1C_Element_UI(uint8_t moduleNumber, uint32_t channel,const char *name,double DefaultNumber=0.0);
	void display_number(double value);
	void display_bool(bool value);
	bool get_bool() const;
	double get_number() const;
protected:
	std::string m_Name;
	double m_DefaultNumber;
	//Check first run to avoid mucking up the SmartDashboard (i.e. only populate with the inital get is called)
	mutable bool m_PutNumber_Used,m_PutBoolUsed;
};

class Control_2C_Element_UI
{
public:
	Control_2C_Element_UI(uint8_t moduleNumber, uint32_t forward_channel, uint32_t reverse_channel,const char *name);
	void display_bool(bool value);
	void display_number(double value);
	bool get_bool(bool defaultvalue=false) const;
	double get_number() const;
protected:
	std::string m_Name;
};

class PWMSpeedController : public Control_1C_Element_UI
{
public:
	PWMSpeedController(PWMSpeedController&&) = default;
	PWMSpeedController& operator=(PWMSpeedController&&) = default;

	virtual void Set(double value, uint8_t syncGroup=0) {m_CurrentVoltage=value; display_number(value);}
	virtual double Get() {return m_CurrentVoltage;}
	virtual void Disable() {}
	//virtual void PIDWrite(float output);
protected:
	PWMSpeedController(uint32_t channel) : Control_1C_Element_UI(0, channel, "PWMSpeedController"),
		m_ModuleNumber(0), m_Channel(channel) {}
	PWMSpeedController(uint8_t moduleNumber, uint32_t channel, const char *name) : Control_1C_Element_UI(moduleNumber, channel, name),
		m_ModuleNumber(moduleNumber), m_Channel(channel) {}
private:
	uint8_t m_ModuleNumber;
	uint32_t m_Channel;
	double m_CurrentVoltage;
};

//Making a global instance tester to ensure there are no duplicate modules instantiated
class InstanceTester_internal;
class InstanceTester
{
public:
	InstanceTester();
	//returns true if successful, asserts if it fails
	bool operator()(const char *category,uint32_t channel);
	bool operator()(const char *category, uint32_t channelA, uint32_t channelB);
private:
	std::shared_ptr<InstanceTester_internal> m_InstanceTester;
};
extern InstanceTester g_InstanceTester;


class Victor : public PWMSpeedController
{
public:
	Victor(int channel) : PWMSpeedController(0, channel, "Victor") 
	{	g_InstanceTester("pwm", channel);
	}
	Victor(uint8_t moduleNumber, uint32_t channel, const char *name) : PWMSpeedController(0, channel, name)
	{	g_InstanceTester("pwm", channel);
	}
	Victor(Victor&&) = default;
	Victor& operator=(Victor&&) = default;
};

class VictorSP : public PWMSpeedController
{
public:
	/**
	 * Constructor for a VictorSP.
	 *
	 * @param channel The PWM channel that the VictorSP is attached to. 0-9 are
	 *                on-board, 10-19 are on the MXP port
	 */
	VictorSP(int channel) : PWMSpeedController(0, channel, "VictorSP")
	{	g_InstanceTester("pwm", channel);
	}
	VictorSP(uint8_t moduleNumber, uint32_t channel, const char *name) : PWMSpeedController(0, channel, name)
	{	g_InstanceTester("pwm", channel);
	}

	VictorSP(VictorSP&&) = default;
	VictorSP& operator=(VictorSP&&) = default;
};

enum ErrorCode : int32_t
{
	OK = 0
};

enum FeedbackDevice
{
	//NOTE: None was removed as it doesn't exist in firmware
	//TODO: Add None to firmware and add None back in
	QuadEncoder = 0,
	//1
	Analog = 2,
	//3
	Tachometer = 4,
	PulseWidthEncodedPosition = 8,

	SensorSum = 9,
	SensorDifference = 10,
	RemoteSensor0 = 11,
	RemoteSensor1 = 12,
	//13
	//14
	SoftwareEmulatedSensor = 15,

	CTRE_MagEncoder_Absolute = PulseWidthEncodedPosition,
	CTRE_MagEncoder_Relative = QuadEncoder,
};

enum RemoteFeedbackDevice
{
	RemoteFeedbackDevice_None = -1,

	RemoteFeedbackDevice_SensorSum = 9,
	RemoteFeedbackDevice_SensorDifference = 10,
	RemoteFeedbackDevice_RemoteSensor0 = 11,
	RemoteFeedbackDevice_RemoteSensor1 = 12,
	//13
	//14
	RemoteFeedbackDevice_SoftwareEmulatedSensor = 15,
};
enum class ControlMode
{
	PercentOutput = 0,
	Position = 1,
	Velocity = 2,
	Current = 3,
	Follower = 5,
	MotionProfile = 6,
	MotionMagic = 7,
	MotionProfileArc = 10,

	Disabled = 15,
};

class VictorSPX : public Control_1C_Element_UI
{
private:
	uint8_t m_ModuleNumber;
	uint32_t m_Channel;
	double m_CurrentVoltage;
public:
	VictorSPX(int channel) : Control_1C_Element_UI(0, channel, "VictorSPX"),m_ModuleNumber(0),m_Channel(channel)
	{
	}
	//for derived classes
	VictorSPX(int channel, const char *name) : Control_1C_Element_UI(0, channel, name), m_ModuleNumber(0), m_Channel(channel)
	{
	}
	VictorSPX(uint8_t moduleNumber, uint32_t channel, const char *name) : Control_1C_Element_UI(0, channel, name), 
		m_ModuleNumber(moduleNumber), m_Channel(channel)
	{
		g_InstanceTester("VictorSPX", channel);
	}

	void Set(ControlMode mode, double Value) 
	{ m_CurrentVoltage = Value; display_number(Value); 
	}
	virtual double GetMotorOutputPercent() { return 0.0; }
	virtual ErrorCode ConfigSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) { return OK; }
	virtual ErrorCode ConfigSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) { return OK; }
};


class Servo : public Control_1C_Element_UI
{
public:
	Servo(uint8_t moduleNumber, uint32_t channel,const char *name) : Control_1C_Element_UI(moduleNumber,channel,name),
		m_ModuleNumber(moduleNumber), m_Channel(channel) 
	{	g_InstanceTester("Servo", channel);
	}
	void Set(float value) {m_CurrentVoltage=value; display_number(value);}
	//void SetOffline();
	float Get()  {return m_CurrentVoltage;}
	//void SetAngle(float angle);
	//float GetAngle();
	//static float GetMaxAngle() { return kMaxServoAngle; };
	//static float GetMinAngle() { return kMinServoAngle; };
private:
	void InitServo() {}
	//float GetServoAngleRange() {return kMaxServoAngle - kMinServoAngle;}

	//static constexpr float kMaxServoAngle = 170.0;
	//static constexpr float kMinServoAngle = 0.0;
	uint8_t m_ModuleNumber;
	uint32_t m_Channel;
	float m_CurrentVoltage;
};

class DigitalInput : public Control_1C_Element_UI
{
public:
	DigitalInput(uint8_t moduleNumber, uint32_t channel,const char *name) : Control_1C_Element_UI(moduleNumber,channel,name,1.0),
		m_ModuleNumber(moduleNumber), m_Channel(channel) 
	{	g_InstanceTester("DigitalInput", channel);
	}
	DigitalInput(uint32_t channel, const char *name="DigitalInput") : Control_1C_Element_UI(0, channel, name, 1.0),
		m_ModuleNumber(0), m_Channel(channel) 
	{	g_InstanceTester("DigitalInput", channel);
	}
	uint32_t Get() {return (uint32_t)get_number();}
	uint32_t GetChannel() {return m_Channel;}
private:
	uint8_t m_ModuleNumber;
	uint32_t m_Channel;
};

class AnalogChannel : public Control_1C_Element_UI
{
public:
	AnalogChannel(uint8_t moduleNumber, uint32_t channel,const char *name) : Control_1C_Element_UI(moduleNumber,channel,name,1.0),
		m_ModuleNumber(moduleNumber), m_Channel(channel) 
	{	g_InstanceTester("AnalogChannel", channel);
	}
	void TimeChange(double dTime_s, double external_value) //only used in simulation
	{	m_DefaultNumber = external_value;
		display_number(external_value);  //also update smartdashboard
	}
	int16_t GetValue() {return (int16_t)get_number();}
	int32_t GetAverageValue()  {return (int32_t)get_number();}
	uint32_t GetChannel()  {return m_Channel;}
private:
	uint8_t m_ModuleNumber;
	uint32_t m_Channel;
};

class DoubleSolenoid : public Control_2C_Element_UI
{
public:
	typedef enum {kOff, kForward, kReverse} Value;
	DoubleSolenoid(uint8_t moduleNumber, uint32_t forwardChannel, uint32_t reverseChannel,const char *name) : 
		Control_2C_Element_UI(moduleNumber,forwardChannel,reverseChannel,name),
		m_ModuleNumber(moduleNumber),m_forwardChannel(forwardChannel),m_reverseChannel(reverseChannel) 
	{	g_InstanceTester("DoubleSolenoid", forwardChannel, reverseChannel);
	}

	DoubleSolenoid(uint32_t forwardChannel, uint32_t reverseChannel, const char *name="DoubleSolenoid") :
		Control_2C_Element_UI(0, forwardChannel, reverseChannel, name),
		m_ModuleNumber(0), m_forwardChannel(forwardChannel), m_reverseChannel(reverseChannel) 
	{	g_InstanceTester("DoubleSolenoid", forwardChannel, reverseChannel);
	}

	virtual void Set(Value value) {m_CurrentValue=value; display_bool(value==kForward);}
	virtual Value Get() {return m_CurrentValue=get_bool(m_CurrentValue==kForward)?kForward:kReverse;}
private:
	uint8_t m_ModuleNumber;
	uint32_t m_forwardChannel; ///< The forward channel on the module to control.
	uint32_t m_reverseChannel; ///< The reverse channel on the module to control.
	Value m_CurrentValue;
};

class Encoder2 : public Control_2C_Element_UI
{
public:
	//Note: Encoder allows two module numbers... we'll skip that support since double solenoid doesn't have it, and we can reuse 2C
	//for both... we can change if needed
	Encoder2(uint8_t ModuleNumber,UINT32 aChannel, UINT32 bChannel,const char *name);
	void TimeChange(double dTime_s,double adjustment_delta); //only used in simulation
	double GetRate2(double dTime_s);
	void Reset2();

	void Start();
	int32_t Get();
	int32_t GetRaw();
	void Reset();
	void Stop();
	double GetDistance();
	double GetRate();
	void SetMinRate(double minRate);
	void SetDistancePerPulse(double distancePerPulse);
	void SetReverseDirection(bool reverseDirection);
private:
	double m_LastDistance;  //keep note of last distance
	double m_Distance;
	double m_LastTime;
	double m_ValueScalar;  //used to reverse direction
};

class Relay : public Control_1C_Element_UI
{
public:
	typedef enum {kOff, kOn, kForward, kReverse} Value;
	typedef enum {kBothDirections, kForwardOnly, kReverseOnly} Direction;

	Relay(uint8_t moduleNumber, uint32_t channel,const char *name) :  Control_1C_Element_UI(moduleNumber,channel,name),
		m_ModuleNumber(moduleNumber), m_Channel(channel) {}
	//virtual ~Relay();

	virtual void Set(Value value) {m_CurrentValue=value; display_number((double)value);}
	virtual Value Get() {return m_CurrentValue=get_number()==0.0?kOff:get_number()>0.0?kForward:kReverse;}
private:
	uint8_t m_ModuleNumber;
	uint32_t m_Channel;
	Value m_CurrentValue;
};

class COMMON_API Compressor : public Control_2C_Element_UI
{
public:
	Compressor(uint32_t pressureSwitchChannel, uint32_t compressorRelayChannel) : 
	  Control_2C_Element_UI(1,pressureSwitchChannel,compressorRelayChannel,"compressor"), m_enabled(false)
	  {
		  display_bool(false);
	  }

	void Start() {display_bool(true); m_enabled=true;}
	void Stop() {display_bool(false); m_enabled=false;}
	bool Enabled() {return m_enabled;}
	//uint32_t GetPressureSwitchValue();
	//void SetRelayValue(Relay::Value relayValue);

private:
	bool m_enabled;
};

class COMMON_API Accelerometer
{
public:
	virtual ~Accelerometer() {};

	enum Range
	{
		kRange_2G = 0,
		kRange_4G = 1,
		kRange_8G = 2,
		kRange_16G = 3
	};

	virtual void SetRange(Range range) = 0;
	virtual double GetX() = 0;
	virtual double GetY() = 0;
	virtual double GetZ() = 0;
};


class BuiltInAccelerometer : public Accelerometer, public Control_1C_Element_UI
{
public:
	BuiltInAccelerometer(Range range = kRange_8G) :	Control_1C_Element_UI(0,0,"accelerometer")
	{
		display_bool(false);
	}

	virtual ~BuiltInAccelerometer();

	// Accelerometer interface
	virtual void SetRange(Range range);
	//I'm not too worried about the implementation of the simulation... just one method is fine
	virtual double GetX()  {return get_number();}
	virtual double GetY() {return 0.0;}
	virtual double GetZ() {return 0.0;}
};
#endif
class RobotDrive_Interface
{
	public:
		//Must have this
		virtual ~RobotDrive_Interface() {}
		enum MotorType
		{
			kFrontLeftMotor = 0,
			kFrontRightMotor = 1,
			kRearLeftMotor = 2,
			kRearRightMotor = 3,
			kCenterLeftMotor = 4,
			kCenterRightMotor = 5
		};
		virtual void SetLeftRightMotorOutputs(float leftOutput, float rightOutput)=0;
		virtual void GetLeftRightMotorOutputs(float &leftOutput, float &rightOutput) = 0;
		virtual void SetInvertedMotor(MotorType motor, bool isInverted);
		virtual void SetExpiration(float timeout);
		virtual float GetExpiration();
		virtual bool IsAlive();
		virtual void StopMotor() = 0;
		virtual bool IsSafetyEnabled();
		virtual void SetSafetyEnabled(bool enabled);
		virtual void GetDescription(char *desc) = 0;
	protected:
		float Limit(float num);
		void Normalize(double *wheelSpeeds);
		void RotateVector(double &x, double &y, double angle);

		int32_t m_invertedMotors[6];
		float m_sensitivity;
		double m_maxOutput;
		bool m_deleteSpeedControllers;

		float m_LeftOutput, m_RightOutput;
		bool m_IsEnabled = false;  //Use to determine if its been enabled for quick disable access
};

class COMMON_API RobotDrive2 : public RobotDrive_Interface
{
public:
	RobotDrive2(PWMSpeedController *frontLeftMotor, PWMSpeedController *rearLeftMotor,PWMSpeedController *frontRightMotor, PWMSpeedController *rearRightMotor,
		PWMSpeedController *centerLeftMotor=nullptr, PWMSpeedController *centerRightMotor=nullptr);
	RobotDrive2(PWMSpeedController &frontLeftMotor, PWMSpeedController &rearLeftMotor,PWMSpeedController &frontRightMotor, PWMSpeedController &rearRightMotor);
	virtual ~RobotDrive2();

	virtual void SetLeftRightMotorOutputs(float leftOutput, float rightOutput);
	virtual void GetLeftRightMotorOutputs(float &leftOutput, float &rightOutput) //I added this one for convenience
	{	leftOutput=m_LeftOutput,rightOutput=m_RightOutput;
	}
	void StopMotor();
	void GetDescription(char *desc);

protected:
	void InitRobotDrive();

	//static const int32_t kMaxNumberOfMotors = 4;
	PWMSpeedController *m_frontLeftMotor;
	PWMSpeedController *m_frontRightMotor;
	PWMSpeedController *m_rearLeftMotor;
	PWMSpeedController *m_rearRightMotor;
	PWMSpeedController *m_centerLeftMotor;
	PWMSpeedController *m_centerRightMotor;
private:
	int32_t GetNumMotors()
	{
		int motors = 0;
		if (m_frontLeftMotor) motors++;
		if (m_frontRightMotor) motors++;
		if (m_rearLeftMotor) motors++;
		if (m_rearRightMotor) motors++;
		if (m_centerLeftMotor) motors++;
		if (m_centerRightMotor) motors++;
		return motors;
	}
};

class COMMON_API RobotDrive_SPX : public RobotDrive_Interface
{
public:
	RobotDrive_SPX(VictorSPX *frontLeftMotor, VictorSPX *rearLeftMotor, VictorSPX *frontRightMotor, VictorSPX *rearRightMotor,
		VictorSPX *centerLeftMotor = nullptr, VictorSPX *centerRightMotor = nullptr);
	RobotDrive_SPX(VictorSPX &frontLeftMotor, VictorSPX &rearLeftMotor, VictorSPX &frontRightMotor, VictorSPX &rearRightMotor);
	virtual ~RobotDrive_SPX();

	virtual void SetLeftRightMotorOutputs(float leftOutput, float rightOutput);
	virtual void GetLeftRightMotorOutputs(float &leftOutput, float &rightOutput) //I added this one for convenience
	{
		leftOutput = m_LeftOutput, rightOutput = m_RightOutput;
	}
	void StopMotor();
	void GetDescription(char *desc);

protected:
	void InitRobotDrive();

	//static const int32_t kMaxNumberOfMotors = 4;
	VictorSPX *m_frontLeftMotor;
	VictorSPX *m_frontRightMotor;
	VictorSPX *m_rearLeftMotor;
	VictorSPX *m_rearRightMotor;
	VictorSPX *m_centerLeftMotor;
	VictorSPX *m_centerRightMotor;
private:
	int32_t GetNumMotors()
	{
		int motors = 0;
		if (m_frontLeftMotor) motors++;
		if (m_frontRightMotor) motors++;
		if (m_rearLeftMotor) motors++;
		if (m_rearRightMotor) motors++;
		if (m_centerLeftMotor) motors++;
		if (m_centerRightMotor) motors++;
		return motors;
	}
};


#define LUT_VALID(x) ((index<x.size()) && (x[index]!=(size_t)-1))
#define IF_LUT(x) if ((index<x.size()) && (x[index]!=(size_t)-1))

#ifdef _Win32
typedef AnalogChannel AnalogInput;
#endif

class COMMON_API RobotControlCommon
{
	public:
		typedef std::vector<size_t> Controls_LUT;
		RobotControlCommon();
		virtual ~RobotControlCommon();

		//PWMSpeedController methods
		__inline double PWMSpeedController_GetCurrentPorV(size_t index) {return LUT_VALID(m_PWMSpeedControllerLUT)?m_PWMSpeedControllers[m_PWMSpeedControllerLUT[index]]->Get() : 0.0;}
		__inline void PWMSpeedController_UpdateVoltage(size_t index,double Voltage) {IF_LUT(m_PWMSpeedControllerLUT) m_PWMSpeedControllers[m_PWMSpeedControllerLUT[index]]->Set((float)Voltage);}
		__inline PWMSpeedController *PWMSpeedController_GetInstance(size_t index) 
		{
			return LUT_VALID(m_PWMSpeedControllerLUT)?m_PWMSpeedControllers[m_PWMSpeedControllerLUT[index]] : NULL;
		}

		//VictorSPX methods
		__inline double VictorSPX_GetCurrentPorV(size_t index) { return LUT_VALID(m_VictorSPX_LUT) ? m_VictorSPXs[m_VictorSPX_LUT[index]]->GetMotorOutputPercent() : 0.0; }
		__inline void VictorSPX_UpdateVoltage(size_t index, double Voltage) { IF_LUT(m_VictorSPX_LUT) m_VictorSPXs[m_VictorSPX_LUT[index]]->Set(ControlMode::PercentOutput, Voltage); }
		__inline VictorSPX *VictorSPX_GetInstance(size_t index)
		{
			return LUT_VALID(m_VictorSPX_LUT) ? m_VictorSPXs[m_VictorSPX_LUT[index]] : NULL;
		}

		//servo methods
		__inline double Servo_GetCurrentPorV(size_t index) {return LUT_VALID(m_ServoLUT)?m_Servos[m_ServoLUT[index]]->Get() : 0.0;}
		//Note: we convert the voltage to the -1 - 1 range by adding 1 and dividing by 2
		__inline void Servo_UpdateVoltage(size_t index,double Voltage) {IF_LUT(m_ServoLUT) m_Servos[m_ServoLUT[index]]->Set((float)((Voltage + 1.0) / 2.0));}
		__inline Servo *Servo_GetInstance(size_t index) {return LUT_VALID(m_ServoLUT)?m_Servos[m_ServoLUT[index]] : NULL;}

		//solenoid methods
		__inline void Solenoid_Open(size_t index,bool Open=true) 
		{	IF_LUT(m_DoubleSolenoidLUT)
			{
				DoubleSolenoid::Value value=Open ? DoubleSolenoid::kForward : DoubleSolenoid::kReverse;
				m_DoubleSolenoids[m_DoubleSolenoidLUT[index]]->Set(value);
			}
		}
		__inline void Solenoid_Close(size_t index,bool Close=true) {Solenoid_Open(index,!Close);}
		__inline bool Solenoid_GetIsOpen(size_t index) const 
		{	return LUT_VALID(m_DoubleSolenoidLUT)?m_DoubleSolenoids[m_DoubleSolenoidLUT[index]]->Get()==DoubleSolenoid::kForward : false;
		}
		__inline bool Solenoid_GetIsClosed(size_t index) const {return !Solenoid_GetIsOpen(index);}
		__inline DoubleSolenoid *Solenoid_GetInstance(size_t index) {return LUT_VALID(m_DoubleSolenoidLUT)?m_DoubleSolenoids[m_DoubleSolenoidLUT[index]] : NULL;}

		//digital input method
		__inline bool BoolSensor_GetState(size_t index) {return LUT_VALID(m_DigitalInputLUT)?m_DigitalInputs[m_DigitalInputLUT[index]]->Get()==0:false;}
		__inline DigitalInput *BoolSensor_GetInstance(size_t index) {return LUT_VALID(m_DigitalInputLUT)?m_DigitalInputs[m_DigitalInputLUT[index]] : NULL;}

		//digital input encoders
		__inline double Encoder_GetRate(size_t index) {return LUT_VALID(m_EncoderLUT)?m_Encoders[m_EncoderLUT[index]]->GetRate():0.0;}
		__inline double Encoder_GetDistance(size_t index) {return LUT_VALID(m_EncoderLUT)?m_Encoders[m_EncoderLUT[index]]->GetDistance():0.0;}

		__inline void Encoder_Start(size_t index) { }
		__inline void Encoder_Stop(size_t index)  { }

		__inline void Encoder_Reset(size_t index) {	IF_LUT(m_EncoderLUT) m_Encoders[m_EncoderLUT[index]]->Reset();}
		__inline void Encoder_SetDistancePerPulse(size_t index,double distancePerPulse) {IF_LUT(m_EncoderLUT) m_Encoders[m_EncoderLUT[index]]->SetDistancePerPulse(distancePerPulse);}
		__inline void Encoder_SetReverseDirection(size_t index,bool reverseDirection)   {IF_LUT(m_EncoderLUT) m_Encoders[m_EncoderLUT[index]]->SetReverseDirection(reverseDirection);}
		#ifdef _Win32
		__inline void Encoder_TimeChange(size_t index,double dTime_s,double adjustment_delta) { IF_LUT(m_EncoderLUT) m_Encoders[m_EncoderLUT[index]]->TimeChange(dTime_s,adjustment_delta);}
		__inline void Analog_TimeChange(size_t index, double dTime_s, double external_value) { IF_LUT(m_AnalogInputLUT) m_AnalogInputs[m_AnalogInputLUT[index]]->TimeChange(dTime_s, external_value); }
		#endif
		__inline Encoder2 *Encoder_GetInstance(size_t index) {return LUT_VALID(m_EncoderLUT)?m_Encoders[m_EncoderLUT[index]] : NULL;}

		//analog channel inputs
		__inline int Analog_GetValue(size_t index) {return LUT_VALID(m_AnalogInputLUT)?(int)m_AnalogInputs[m_AnalogInputLUT[index]]->GetValue():0;}
		__inline int Analog_GetAverageValue(size_t index) {return LUT_VALID(m_AnalogInputLUT)?(int)m_AnalogInputs[m_AnalogInputLUT[index]]->GetAverageValue():0;}

		void TranslateToRelay(size_t index,double Voltage);
		Compressor *CreateCompressor();
		__inline void DestroyCompressor(Compressor *instance) {delete instance;}

		__inline Accelerometer *CreateBuiltInAccelerometer()
		{
			return new BuiltInAccelerometer();
		}
		__inline void DestroyBuiltInAccelerometer(Accelerometer *instance) {delete instance;}
		//Give ability to have controls be created externally
		void SetExternalPWMSpeedControllerHook(std::function<void *(size_t, size_t, const char *,const char *,bool &)> callback) { m_ExternalPWMSpeedController = callback; }
	protected:
		virtual void RobotControlCommon_Initialize(const Control_Assignment_Properties &props);
		//Override by derived class
		virtual size_t RobotControlCommon_Get_PWMSpeedController_EnumValue(const char *name) const =0;
		virtual size_t RobotControlCommon_Get_DigitalInput_EnumValue(const char *name) const =0;
		virtual size_t RobotControlCommon_Get_AnalogInput_EnumValue(const char *name) const =0;
		virtual size_t RobotControlCommon_Get_DoubleSolenoid_EnumValue(const char *name) const =0;
	private:
		Control_Assignment_Properties m_Props;  //cache a copy of the assignment props
		std::vector<PWMSpeedController *> m_PWMSpeedControllers;
		std::vector<VictorSPX *> m_VictorSPXs;
		std::vector<Servo * > m_Servos;
		std::vector<Relay *> m_Relays;
		std::vector<DigitalInput *> m_DigitalInputs;
		std::vector<AnalogInput *> m_AnalogInputs;
		std::vector<DoubleSolenoid *> m_DoubleSolenoids;
		std::vector<Encoder2 *> m_Encoders;

		Controls_LUT m_PWMSpeedControllerLUT,m_VictorSPX_LUT,m_ServoLUT,m_RelayLUT,m_DigitalInputLUT,m_AnalogInputLUT,m_DoubleSolenoidLUT,m_EncoderLUT;
		std::function<void *(size_t,size_t,const char *,const char *,bool &)> m_ExternalPWMSpeedController;
};
}