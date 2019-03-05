#if 1

#ifndef _Win32
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#endif

#include "../Base/Base_Includes.h"
#include <math.h>
#include <assert.h>
#include <map>
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
//#include "AI_Base_Controller.h"
//#include "Vehicle_Drive.h"
//#include "PIDController.h"
//#include "Poly.h"
//#include "Robot_Control_Interface.h"
//#include "Rotary_System.h"

#include "../Base/Joystick.h"
#include "../Base/JoystickBinder.h"
//TODO see if I should move Encoder2
#ifndef _Win32
#include "InOut_Interface.h"
#endif
#include "Debug.h"

#include "Robot_Control_Common.h"
#include "SmartDashboard.h"

using namespace frc;



#ifdef _Win32
namespace frc
{
class InstanceTester_internal
{
private:
	using channel_list=std::vector<int>;
	std::map<std::string, channel_list, std::greater<std::string>> m_database;
	using db_iter = std::map<std::string, channel_list, std::greater<std::string>>::iterator;
	using const_db_iter = std::map<std::string, channel_list, std::greater<std::string>>::const_iterator;

public:
	bool Test(const char *category, uint32_t channel)
	{
		bool ret = true;
		db_iter test = m_database.find(category);
		if (test != m_database.end())
		{
			//linear search the channels
			const channel_list &channels = test->second;
			for (auto i : channels)
			{
				if (i == channel)
				{
					printf("Duplicate %s[%d]", category, channel);
					assert(false);
					ret = false;
				}
			}
		}
		else
			m_database[category].push_back(channel);
		return ret;
	}
	bool Test2(const char *category, uint32_t channelA, uint32_t channelB)
	{
		bool ret = true;
		db_iter test = m_database.find(category);
		if (test != m_database.end())
		{
			//linear search the channels
			const channel_list &channels = test->second;
			for (auto i : channels)
			{
				if ((i == channelA)|| (i == channelB))
				{
					printf("Duplicate %s[%d,%d]", category, channelA, channelB);
					assert(false);
					ret = false;
				}
			}
		}
		else
		{
			m_database[category].push_back(channelA);
			m_database[category].push_back(channelB);
		}
		return ret;
	}
};
}

frc::InstanceTester::InstanceTester()
{
	m_InstanceTester=std::make_shared<InstanceTester_internal>();
}
bool frc::InstanceTester::operator()(const char *category, uint32_t channel)
{
	return m_InstanceTester->Test(category, channel);
}
bool frc::InstanceTester::operator()(const char *category, uint32_t channelA, uint32_t channelB)
{
	return m_InstanceTester->Test2(category, channelA, channelB);
}

frc::InstanceTester frc::g_InstanceTester;
#endif

  /***********************************************************************************************************************************/
 /*													Control_Assignment_Properties													*/
/***********************************************************************************************************************************/

static void LoadControlElement_1C_Internal(Scripting::Script& script,Control_Assignment_Properties::Controls_1C &Output,const char *type)
{
	typedef Control_Assignment_Properties::Control_Element_1C Control_Element_1C;
	const char* err=NULL;
	const char* fieldtable_err=NULL;
	char Buffer[128];
	size_t index=1;  //keep the lists cardinal in LUA
	do 
	{
		sprintf(Buffer,"id_%d",index);
		fieldtable_err = script.GetFieldTable(Buffer);
		if (!fieldtable_err)
		{
			Control_Element_1C newElement;
			{
				newElement.type = type;
				double fTest;
				err = script.GetField("channel",NULL,NULL,&fTest);
				assert(!err);

				newElement.Channel=(size_t)fTest-1;  //make ordinal

				err = script.GetField("name",&newElement.name,NULL,NULL);
				assert(!err);
				err = script.GetField("module",NULL,NULL,&fTest);

				newElement.Module=(err)?0:(size_t)fTest;
				//assert(newElement.Module!=0);  //note: All module parameters are all ordinal for roboRIO
			}
			Output.push_back(newElement);
			script.Pop();
			index++;
		}
	} while (!fieldtable_err);

}

static void LoadControlElement_2C_Internal(Scripting::Script& script,Control_Assignment_Properties::Controls_2C &Output)
{
	typedef Control_Assignment_Properties::Control_Element_2C Control_Element_2C;
	const char* err=NULL;
	const char* fieldtable_err=NULL;
	char Buffer[128];
	size_t index=1;  //keep the lists cardinal in LUA
	do 
	{
		sprintf(Buffer,"id_%d",index);
		fieldtable_err = script.GetFieldTable(Buffer);
		if (!fieldtable_err)
		{
			Control_Element_2C newElement;
			{
				double fTest;
				err = script.GetField("forward_channel",NULL,NULL,&fTest);
				if (err)
					err=script.GetField("a_channel",NULL,NULL,&fTest);
				assert(!err);

				newElement.ForwardChannel=(size_t)fTest-1;  //make ordinal

				err = script.GetField("reverse_channel",NULL,NULL,&fTest);
				if (err)
					err=script.GetField("b_channel",NULL,NULL,&fTest);
				assert(!err);

				newElement.ReverseChannel=(size_t)fTest-1;  //make ordinal
				err = script.GetField("name",&newElement.name,NULL,NULL);
				assert(!err);
				err = script.GetField("module",NULL,NULL,&fTest);

				newElement.Module=(err)?0:(size_t)fTest;
				//assert(newElement.Module!=0);  //note: All module parameters are all ordinal for roboRIO
			}
			Output.push_back(newElement);
			script.Pop();
			index++;
		}
	} while (!fieldtable_err);
}

void Control_Assignment_Properties::LoadFromScript(Scripting::Script& script)
{
	const char* err=NULL;
	err = script.GetFieldTable("control_assignments");
	if (!err) 
	{
		err = script.GetFieldTable("victor");
		if (!err)
		{
			LoadControlElement_1C_Internal(script,m_PWMSpeedControllers,"Victor");
			script.Pop();
		}
		err = script.GetFieldTable("victor_sp");
		if (!err)
		{
			LoadControlElement_1C_Internal(script, m_PWMSpeedControllers, "VictorSP");
			script.Pop();
		}
		err = script.GetFieldTable("victor_spx");
		if (!err)
		{
			LoadControlElement_1C_Internal(script, m_VictorSPXs, "VictorSPX");
			script.Pop();
		}
		err = script.GetFieldTable("servo");
		if (!err)
		{
			LoadControlElement_1C_Internal(script,m_Servos,"servo");
			script.Pop();
		}
		err = script.GetFieldTable("relay");
		if (!err)
		{
			LoadControlElement_1C_Internal(script,m_Relays,"relay");
			script.Pop();
		}
		err = script.GetFieldTable("digital_input");
		if (!err)
		{
			LoadControlElement_1C_Internal(script,m_Digital_Inputs, "digital_input");
			script.Pop();
		}

		err = script.GetFieldTable("analog_input");
		if (!err)
		{
			LoadControlElement_1C_Internal(script,m_Analog_Inputs, "analog_input");
			script.Pop();
		}

		err = script.GetFieldTable("double_solenoid");
		if (!err)
		{
			LoadControlElement_2C_Internal(script,m_Double_Solenoids);
			script.Pop();
		}
		err = script.GetFieldTable("digital_input_encoder");
		if (!err)
		{
			LoadControlElement_2C_Internal(script,m_Encoders);
			script.Pop();
		}
		err = script.GetFieldTable("compressor");
		if (!err)
		{
			double fTest;
			err = script.GetField("relay",NULL,NULL,&fTest);
			assert(!err);
			m_Compressor_Relay=(size_t)fTest;
			err = script.GetField("limit",NULL,NULL,&fTest);
			assert(!err);
			m_Compressor_Limit=(size_t)fTest;
			script.Pop();
		}
		else
			m_Compressor_Relay=8,m_Compressor_Limit=14;

		script.Pop();
	}
}


  /***********************************************************************************************************************************/
 /*														RobotControlCommon															*/
/***********************************************************************************************************************************/
void *DefaultExternalControlHook(size_t Module, size_t Channel, const char *Name,const char *Type,bool &DoNotCreate)
{
	return nullptr;
}
RobotControlCommon::RobotControlCommon()
{
	m_ExternalPWMSpeedController = DefaultExternalControlHook;
}

RobotControlCommon::~RobotControlCommon()
{

}

template <class T, class BaseT>
__inline void Initialize_1C_LUT(const Control_Assignment_Properties::Controls_1C &control_props,std::vector<BaseT *> &controls,
	RobotControlCommon::Controls_LUT &control_LUT,RobotControlCommon *instance,
	size_t (RobotControlCommon::*delegate)(const char *name) const, std::function<void *(size_t, size_t, const char *,const char *,bool &DoNotCreate)>External)
{
	//typedef Control_Assignment_Properties::Controls_1C Controls_1C;
	typedef Control_Assignment_Properties::Control_Element_1C Control_Element_1C;
	for (size_t i=0;i<control_props.size();i++)
	{
		const Control_Element_1C &element=control_props[i];
		//ensure this name exists in the list
		size_t enumIndex=(instance->*delegate)(element.name.c_str());
		//The name may not exist in this list (it may be a name specific to the robot)... in which case there is no work to do
		if (enumIndex==(size_t)-1)
			continue;
		bool DoNotCreate = false;
		//See if we have an external hook to allocate this control already
		T *NewElement=(T *)External(element.Module, element.Channel, element.name.c_str(),element.type.c_str(), DoNotCreate);
		if (DoNotCreate)
			continue;
		if (NewElement == nullptr)
		{
			//create the new Control
			#ifdef _Win32
			NewElement = new T((uint8_t)element.Module, (uint32_t)element.Channel, element.name.c_str());  //adding name for UI
			#else
			//quick debug when things are not working
			printf("new %s as %d\n", element.name.c_str(), element.Channel);
			NewElement = new T(element.Channel);
			#endif
		}
		else
			printf("external %s=%s[%d]\n", element.name.c_str(), element.type.c_str(), element.Channel);  //keep track of things which are external

		const size_t LUT_index=controls.size(); //get size before we add it in
		//const size_t PopulationIndex=controls.size();  //get the ordinal value before we add it
		controls.push_back(NewElement);  //add it to our list of PWMSpeedControllers
		//Now to work out the new LUT
		//our LUT is the EnumIndex position set to the value of i... make sure we have the slots created
		//Note: with the more board we can have more PWMSpeedControllers, so I've increased this to 20... also this index does not represent pin count, but is a separate index
		//that is mapped to the pin count index
		assert(enumIndex<20);  //sanity check we have a limit to how many PWMSpeedControllers we have
		while(control_LUT.size()<=enumIndex)
			control_LUT.push_back(-1);  //fill with -1 as a way to indicate nothing is located for that slot
		control_LUT[enumIndex]=LUT_index;
	}
}

template <class T>
__inline void Initialize_2C_LUT(const Control_Assignment_Properties::Controls_2C &control_props,std::vector<T *> &controls,
		RobotControlCommon::Controls_LUT &control_LUT,RobotControlCommon *instance,
		size_t (RobotControlCommon::*delegate)(const char *name) const, 
		std::function<void *(size_t, size_t, const char *, const char *, bool &DoNotCreate)>External)
{
	//typedef Control_Assignment_Properties::Controls_2C Controls_2C;
	typedef Control_Assignment_Properties::Control_Element_2C Control_Element_2C;
	for (size_t i=0;i<control_props.size();i++)
	{
		const Control_Element_2C &element=control_props[i];
		//ensure this name exists in the list
		size_t enumIndex=(instance->*delegate)(element.name.c_str());
		//The name may not exist in this list (it may be a name specific to the robot)... in which case there is no work to do
		if (enumIndex==(size_t)-1)
			continue;
		bool DoNotCreate = false;
		//See if we have an external hook to allocate this control already
		T *NewElement = (T *)External(element.ForwardChannel, element.ReverseChannel, element.name.c_str(), element.name.c_str(), DoNotCreate);
		if (DoNotCreate)
			continue;
		if (NewElement == nullptr)
		{
			//create the new Control
			#ifdef _Win32
			NewElement = new T((uint8_t)element.Module, (uint32_t)element.ForwardChannel, (uint32_t)element.ReverseChannel, element.name.c_str());
			#else
			//quick debug when things are not working
			printf("new %s as %d, %d\n", element.name.c_str(), element.ForwardChannel, element.ReverseChannel);
			NewElement = new T(element.Module, element.ForwardChannel, element.ReverseChannel);
			#endif
		}
		else
			printf("external %s=[%d,%d]\n", element.name.c_str(), element.ForwardChannel, element.ReverseChannel);  //keep track of things which are external

		const size_t LUT_index=controls.size(); //get size before we add it in
		//const size_t PopulationIndex=controls.size();  //get the ordinal value before we add it
		controls.push_back(NewElement);  //add it to our list of PWMSpeedControllers
		//Now to work out the new LUT
		//our LUT is the EnumIndex position set to the value of i... make sure we have the slots created
		assert(enumIndex<20);  //sanity check we have a limit to how many PWMSpeedControllers we have
		while(control_LUT.size()<=enumIndex)
			control_LUT.push_back(-1);  //fill with -1 as a way to indicate nothing is located for that slot
		control_LUT[enumIndex]=LUT_index;
	}
}


void RobotControlCommon::RobotControlCommon_Initialize(const Control_Assignment_Properties &props)
{
	m_Props=props;
	#ifdef _Win32
	typedef Control_Assignment_Properties::Controls_1C Controls_1C;
	typedef Control_Assignment_Properties::Control_Element_1C Control_Element_1C;
	typedef Control_Assignment_Properties::Controls_2C Controls_2C;
	typedef Control_Assignment_Properties::Control_Element_2C Control_Element_2C;
	#endif
	//create control elements and their LUT's
	//Note: PWMSpeedControllers,Servos, and Relays share the PWM slots; therefore they share the same enumeration, and can be used interchangeably in high level code
	//PWMSpeedControllers
	Initialize_1C_LUT<Victor,PWMSpeedController>(props.GetPWMSpeedControllers(),m_PWMSpeedControllers,m_PWMSpeedControllerLUT,this,&RobotControlCommon::RobotControlCommon_Get_PWMSpeedController_EnumValue, 
		[&](size_t Module, size_t Channel, const char *Name, const char *Type,bool &DoNotCreate)
		{
			//Ok Explanation is needed here... first we try the external use case, if it returns a pointer we are good
			void *ret = m_ExternalPWMSpeedController(Module,Channel,Name,Type,DoNotCreate);
			//If not... we create the correct control based off of its type
			if ((ret == nullptr)&&(DoNotCreate==false))
			{
				//Currently we have either Victor or VictorSP
				if (strcmp(Type, "VictorSP") == 0)
				{
					#ifdef _Win32
					std::string NameToUse = "VictorSP_";  //I could assign Type, but I want to monitor the logic path
					NameToUse += Name;
					ret = new VictorSP((uint8_t)Module, (uint32_t)Channel, NameToUse.c_str());  //adding name for UI
					#else
					//quick debug when things are not working
					printf("new %s as %d\n", Name, Channel);
					ret = new VictorSP((int)Channel);
					#endif
				}
				else if (strcmp(Type, "VictorSPX") == 0)
				{
					#ifdef _Win32
					std::string NameToUse = "VictorSPX_";  //I could assign Type, but I want to monitor the logic path
					NameToUse += Name;
					ret = new VictorSPX((uint32_t)Channel, NameToUse.c_str());  //adding name for UI
					#else
					//quick debug when things are not working
					printf("new %s as %d\n", Name, Channel);
					ret = new VictorSPX((int)Channel);
					#endif

				}
				else
				{
					assert(strcmp(Type, "Victor") == 0);  //better be a victor
					ret = nullptr;  //let it fall back to default implmentation 
				}
			}
			return	ret;
		}
		);

	Initialize_1C_LUT<VictorSPX, VictorSPX>(props.GetVictorSPXs(),m_VictorSPXs,m_VictorSPX_LUT,this,&RobotControlCommon::RobotControlCommon_Get_PWMSpeedController_EnumValue,
		[&](size_t Module, size_t Channel, const char *Name, const char *Type,bool &DoNotCreate)
		{
			//Ok Explanation is needed here... first we try the external use case, if it returns a pointer we are good
			void *ret = m_ExternalPWMSpeedController(Module,Channel,Name,Type,DoNotCreate);
			//If not... we create the correct control based off of its type
			if ((ret == nullptr)&&(DoNotCreate==false))
			{
				//Currently we have either Victor or VictorSP
				if (strcmp(Type, "VictorSPX") == 0)
				{
					#ifdef _Win32
					std::string NameToUse = "VictorSPX_";  //I could assign Type, but I want to monitor the logic path
					NameToUse += Name;
					ret = new VictorSPX((uint32_t)Channel, NameToUse.c_str());  //adding name for UI
					#else
					//quick debug when things are not working
					printf("new %s as %d\n", Name, Channel);
					ret = new VictorSPX((int)Channel);
					#endif

				}
				else
				{
					assert(false); 
					ret = nullptr;  //let it fall back to default implmentation 
				}
			}
			return	ret;
		}
		);

	//servos
	Initialize_1C_LUT<Servo, Servo>(props.GetServos(),m_Servos,m_ServoLUT,this,&RobotControlCommon::RobotControlCommon_Get_PWMSpeedController_EnumValue, DefaultExternalControlHook);
	//relays
	Initialize_1C_LUT<Relay, Relay>(props.GetRelays(),m_Relays,m_RelayLUT,this,&RobotControlCommon::RobotControlCommon_Get_PWMSpeedController_EnumValue, DefaultExternalControlHook);
	//double solenoids
	Initialize_2C_LUT<DoubleSolenoid>(props.GetDoubleSolenoids(),m_DoubleSolenoids,m_DoubleSolenoidLUT,this,&RobotControlCommon::RobotControlCommon_Get_DoubleSolenoid_EnumValue, 
		[&](size_t Forward, size_t Reverse, const char *Name, const char *Type, bool &DoNotCreate)
		{
		void *ret = m_ExternalPWMSpeedController(Forward, Reverse, Name, Type, DoNotCreate);  //we'll just use the same callback since it can fit
		return ret;
		});
	//digital inputs
	Initialize_1C_LUT<DigitalInput, DigitalInput>(props.GetDigitalInputs(),m_DigitalInputs,m_DigitalInputLUT,this,&RobotControlCommon::RobotControlCommon_Get_DigitalInput_EnumValue, DefaultExternalControlHook);
	//analog inputs
	Initialize_1C_LUT<AnalogInput, AnalogInput>(props.GetAnalogInputs(),m_AnalogInputs,m_AnalogInputLUT,this,&RobotControlCommon::RobotControlCommon_Get_AnalogInput_EnumValue, DefaultExternalControlHook);
	//encoders
	Initialize_2C_LUT<Encoder2>(props.GetEncoders(),m_Encoders,m_EncoderLUT,this,&RobotControlCommon::RobotControlCommon_Get_PWMSpeedController_EnumValue, DefaultExternalControlHook);
}


void RobotControlCommon::TranslateToRelay(size_t index,double Voltage)
{
	IF_LUT(m_RelayLUT)
	{
		Relay::Value value=Relay::kOff;  //*NEVER* want both on!
		const double Threshold=0.08;  //This value is based on dead voltage for arm... feel free to adjust, but keep high enough to avoid noise

		if (Voltage>Threshold)
			value=Relay::kForward;
		else if (Voltage<-Threshold)
			value=Relay::kReverse;

		m_Relays[m_RelayLUT[index]]->Set(value);
	}
}

Compressor *RobotControlCommon::CreateCompressor()
{
	Compressor *ret = nullptr;
	//Don't create the compressor until we ask for it in the lua
	if (m_Props.GetCompressorLimit() != -1)
	{
		#ifdef _Win32
		ret=new Compressor((uint32_t)m_Props.GetCompressorLimit(), (uint32_t)m_Props.GetCompressorRelay());
		#else
		ret=new Compressor(0);  //This is now the PCM node ID
		#endif
	}
	return ret;
}


#ifdef _Win32
  /***********************************************************************************************************************************/
 /*																Encoder2															*/
/***********************************************************************************************************************************/

Encoder2::Encoder2(uint8_t ModuleNumber,UINT32 aChannel, UINT32 bChannel,const char *name) : 
	Control_2C_Element_UI(ModuleNumber,aChannel,bChannel,name), m_LastDistance(0.0), m_Distance(0.0),m_LastTime(0.010), m_ValueScalar(1.0)
{
	m_Name+="_encoder";
}

void Encoder2::TimeChange(double dTime_s,double adjustment_delta)
{
	m_LastTime=dTime_s;
	m_Distance+=(adjustment_delta * m_ValueScalar);
	display_number(m_Distance);
}
void Encoder2::Reset2()
{
	m_LastDistance=0.0;
	Reset();
}

double Encoder2::GetRate2(double dTime_s)
{
	//Using distance will yield the same rate as GetRate, without precision loss to GetRaw()
	const double CurrentDistance=GetDistance();
	const double delta=CurrentDistance - m_LastDistance;
	m_LastDistance=CurrentDistance;
	return (delta/dTime_s) * m_ValueScalar;
}

void Encoder2::Start() {}
int32_t Encoder2::Get() {return 0;}
int32_t Encoder2::GetRaw() {return 0;}
void Encoder2::Reset() 
{
	m_Distance=0;
}
void Encoder2::Stop() {}
double Encoder2::GetDistance() {return m_Distance;}
double Encoder2::GetRate() {return GetRate2(m_LastTime);}
void Encoder2::SetMinRate(double minRate) {}
void Encoder2::SetDistancePerPulse(double distancePerPulse) {}
void Encoder2::SetReverseDirection(bool reverseDirection) 
{
	m_ValueScalar= reverseDirection?-1.0:1.0;
}
#endif //_Win32

  /***********************************************************************************************************************************/
 /*																RobotDrive2															*/
/***********************************************************************************************************************************/

const int32_t kMaxNumberOfMotors=6;

void RobotDrive2::InitRobotDrive() {
	m_frontLeftMotor = NULL;
	m_frontRightMotor = NULL;
	m_rearRightMotor = NULL;
	m_rearLeftMotor = NULL;
	m_centerLeftMotor = NULL;
	m_centerRightMotor = NULL;
	m_sensitivity = 0.5;
	m_maxOutput = 1.0;
	m_LeftOutput=0.0,m_RightOutput=0.0;
}

RobotDrive2::RobotDrive2(PWMSpeedController *frontLeftMotor, PWMSpeedController *rearLeftMotor,
						PWMSpeedController *frontRightMotor, PWMSpeedController *rearRightMotor,
						PWMSpeedController *centerLeftMotor, PWMSpeedController *centerRightMotor)
{
	InitRobotDrive();
	if (frontLeftMotor == NULL || rearLeftMotor == NULL || frontRightMotor == NULL || rearRightMotor == NULL)
	{
		//wpi_setWPIError(NullParameter);
		//assert(false);
		m_IsEnabled = false;
		return;
	}
	else
		m_IsEnabled = true;
	m_frontLeftMotor = frontLeftMotor;
	m_rearLeftMotor = rearLeftMotor;
	m_frontRightMotor = frontRightMotor;
	m_rearRightMotor = rearRightMotor;
	m_centerLeftMotor = centerLeftMotor;
	m_centerRightMotor = centerRightMotor;
	for (int32_t i=0; i < kMaxNumberOfMotors; i++)
	{
		m_invertedMotors[i] = 1;
	}
	m_deleteSpeedControllers = false;
}

RobotDrive2::RobotDrive2(PWMSpeedController &frontLeftMotor, PWMSpeedController &rearLeftMotor,
						PWMSpeedController &frontRightMotor, PWMSpeedController &rearRightMotor)
{
	InitRobotDrive();
	m_frontLeftMotor = &frontLeftMotor;
	m_rearLeftMotor = &rearLeftMotor;
	m_frontRightMotor = &frontRightMotor;
	m_rearRightMotor = &rearRightMotor;
	for (int32_t i=0; i < kMaxNumberOfMotors; i++)
	{
		m_invertedMotors[i] = 1;
	}
	m_deleteSpeedControllers = false;
}

RobotDrive2::~RobotDrive2()
{
	if (m_deleteSpeedControllers)
	{
		delete m_frontLeftMotor;
		delete m_rearLeftMotor;
		delete m_frontRightMotor;
		delete m_rearRightMotor;
		delete m_centerLeftMotor;
		delete m_centerRightMotor;
	}
}

void RobotDrive2::SetLeftRightMotorOutputs(float leftOutput, float rightOutput)
{
	if (!m_IsEnabled) return;
	//this is added for convenience in simulation
	m_LeftOutput=leftOutput,m_RightOutput=rightOutput;

	assert(m_rearLeftMotor != NULL && m_rearRightMotor != NULL);

	//syncGroup no longer used
	//uint8_t syncGroup = 0x80;

	m_frontLeftMotor->Set((float)(Limit(leftOutput) * m_invertedMotors[kFrontLeftMotor] * m_maxOutput));
	m_rearLeftMotor->Set((float)(Limit(leftOutput) * m_invertedMotors[kRearLeftMotor] * m_maxOutput));

	m_frontRightMotor->Set((float)(-Limit(rightOutput) * m_invertedMotors[kFrontRightMotor] * m_maxOutput));
	m_rearRightMotor->Set((float)(-Limit(rightOutput) * m_invertedMotors[kRearRightMotor] * m_maxOutput));

	if (m_centerLeftMotor)
		m_centerLeftMotor->Set((float)(Limit(leftOutput) * m_invertedMotors[kCenterLeftMotor] * m_maxOutput));
	if (m_centerRightMotor)
		m_centerRightMotor->Set((float)(Limit(rightOutput) * m_invertedMotors[kCenterRightMotor] * m_maxOutput));

	//TODO should eventually update to reflect this, but this shouldn't affect the functionality
	#if 0
	m_frontLeftMotor->Set(Limit(leftOutput) * m_maxOutput);
	m_rearLeftMotor->Set(Limit(leftOutput) * m_maxOutput);
	m_frontRightMotor->Set(-Limit(rightOutput) * m_maxOutput);
	m_rearRightMotor->Set(-Limit(rightOutput) * m_maxOutput);
	#endif
	//CANJaguar::UpdateSyncGroup(syncGroup);  ah ha... sync group only works with CAN / Jaguar
}

float RobotDrive_Interface::Limit(float num)
{
	if (num > 1.0)
	{
		return 1.0;
	}
	if (num < -1.0)
	{
		return -1.0;
	}
	return num;
}

void RobotDrive_Interface::Normalize(double *wheelSpeeds)
{
	double maxMagnitude = fabs(wheelSpeeds[0]);
	int32_t i;
	for (i=1; i<kMaxNumberOfMotors; i++)
	{
		double temp = fabs(wheelSpeeds[i]);
		if (maxMagnitude < temp) maxMagnitude = temp;
	}
	if (maxMagnitude > 1.0)
	{
		for (i=0; i<kMaxNumberOfMotors; i++)
		{
			wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
		}
	}
}

void RobotDrive_Interface::RotateVector(double &x, double &y, double angle)
{
	double cosA = cos(angle * (3.14159 / 180.0));
	double sinA = sin(angle * (3.14159 / 180.0));
	double xOut = x * cosA - y * sinA;
	double yOut = x * sinA + y * cosA;
	x = xOut;
	y = yOut;
}

void RobotDrive_Interface::SetInvertedMotor(MotorType motor, bool isInverted)
{
	if (motor < 0 || motor > 3)
	{
		//wpi_setWPIError(InvalidMotorIndex);
		assert(false);
		return;
	}
	m_invertedMotors[motor] = isInverted ? -1 : 1;
}

void RobotDrive_Interface::SetExpiration(float timeout){}

float RobotDrive_Interface::GetExpiration()
{
	return 0.0;
}

bool RobotDrive_Interface::IsAlive()
{
	return true;
}

bool RobotDrive_Interface::IsSafetyEnabled()
{
	return true;
}

void RobotDrive_Interface::SetSafetyEnabled(bool enabled) {}

void RobotDrive2::GetDescription(char *desc)
{
	sprintf(desc, "RobotDrive2");
}

void RobotDrive2::StopMotor()
{
	if (m_frontLeftMotor != NULL) m_frontLeftMotor->Disable();
	if (m_frontRightMotor != NULL) m_frontRightMotor->Disable();
	if (m_rearLeftMotor != NULL) m_rearLeftMotor->Disable();
	if (m_rearRightMotor != NULL) m_rearRightMotor->Disable();
}

  /***********************************************************************************************************************************/
 /*																RobotDrive_SPX														*/
/***********************************************************************************************************************************/

//const int32_t kMaxNumberOfMotors=6;

void RobotDrive_SPX::InitRobotDrive() {
	m_frontLeftMotor = NULL;
	m_frontRightMotor = NULL;
	m_rearRightMotor = NULL;
	m_rearLeftMotor = NULL;
	m_centerLeftMotor = NULL;
	m_centerRightMotor = NULL;
	m_sensitivity = 0.5;
	m_maxOutput = 1.0;
	m_LeftOutput=0.0,m_RightOutput=0.0;
}

RobotDrive_SPX::RobotDrive_SPX(VictorSPX *frontLeftMotor, VictorSPX *rearLeftMotor,
						VictorSPX *frontRightMotor, VictorSPX *rearRightMotor,
						VictorSPX *centerLeftMotor, VictorSPX *centerRightMotor)
{
	InitRobotDrive();
	if (frontLeftMotor == NULL || rearLeftMotor == NULL || frontRightMotor == NULL || rearRightMotor == NULL)
	{
		//wpi_setWPIError(NullParameter);
		//assert(false);
		m_IsEnabled = false;
		return;
	}
	else
		m_IsEnabled = true;
	m_frontLeftMotor = frontLeftMotor;
	m_rearLeftMotor = rearLeftMotor;
	m_frontRightMotor = frontRightMotor;
	m_rearRightMotor = rearRightMotor;
	m_centerLeftMotor = centerLeftMotor;
	m_centerRightMotor = centerRightMotor;
	for (int32_t i=0; i < kMaxNumberOfMotors; i++)
	{
		m_invertedMotors[i] = 1;
	}
	m_deleteSpeedControllers = false;
}

RobotDrive_SPX::RobotDrive_SPX(VictorSPX &frontLeftMotor, VictorSPX &rearLeftMotor,
						VictorSPX &frontRightMotor, VictorSPX &rearRightMotor)
{
	InitRobotDrive();
	m_frontLeftMotor = &frontLeftMotor;
	m_rearLeftMotor = &rearLeftMotor;
	m_frontRightMotor = &frontRightMotor;
	m_rearRightMotor = &rearRightMotor;
	for (int32_t i=0; i < kMaxNumberOfMotors; i++)
	{
		m_invertedMotors[i] = 1;
	}
	m_deleteSpeedControllers = false;
}

RobotDrive_SPX::~RobotDrive_SPX()
{
	if (m_deleteSpeedControllers)
	{
		delete m_frontLeftMotor;
		delete m_rearLeftMotor;
		delete m_frontRightMotor;
		delete m_rearRightMotor;
		delete m_centerLeftMotor;
		delete m_centerRightMotor;
	}
}

void RobotDrive_SPX::SetLeftRightMotorOutputs(float leftOutput, float rightOutput)
{
	if (!m_IsEnabled) return;
	//this is added for convenience in simulation
	m_LeftOutput=leftOutput,m_RightOutput=rightOutput;

	assert(m_rearLeftMotor != NULL && m_rearRightMotor != NULL);

	//syncGroup no longer used
	//uint8_t syncGroup = 0x80;

	m_frontLeftMotor->Set(ControlMode::PercentOutput, (float)(Limit(leftOutput) * m_invertedMotors[kFrontLeftMotor] * m_maxOutput));
	m_rearLeftMotor->Set(ControlMode::PercentOutput, (float)(Limit(leftOutput) * m_invertedMotors[kRearLeftMotor] * m_maxOutput));

	m_frontRightMotor->Set(ControlMode::PercentOutput, (float)(-Limit(rightOutput) * m_invertedMotors[kFrontRightMotor] * m_maxOutput));
	m_rearRightMotor->Set(ControlMode::PercentOutput, (float)(-Limit(rightOutput) * m_invertedMotors[kRearRightMotor] * m_maxOutput));

	if (m_centerLeftMotor)
		m_centerLeftMotor->Set(ControlMode::PercentOutput, (float)(Limit(leftOutput) * m_invertedMotors[kCenterLeftMotor] * m_maxOutput));
	if (m_centerRightMotor)
		m_centerRightMotor->Set(ControlMode::PercentOutput, (float)(Limit(rightOutput) * m_invertedMotors[kCenterRightMotor] * m_maxOutput));

	//TODO should eventually update to reflect this, but this shouldn't affect the functionality
	#if 0
	m_frontLeftMotor->Set(Limit(leftOutput) * m_maxOutput);
	m_rearLeftMotor->Set(Limit(leftOutput) * m_maxOutput);
	m_frontRightMotor->Set(-Limit(rightOutput) * m_maxOutput);
	m_rearRightMotor->Set(-Limit(rightOutput) * m_maxOutput);
	#endif
	//CANJaguar::UpdateSyncGroup(syncGroup);  ah ha... sync group only works with CAN / Jaguar
}

void RobotDrive_SPX::GetDescription(char *desc)
{
	sprintf(desc, "RobotDrive_SPX");
}

void RobotDrive_SPX::StopMotor()
{
	if (m_frontLeftMotor != NULL) m_frontLeftMotor->Set(ControlMode::PercentOutput, 0.0);
	if (m_frontRightMotor != NULL) m_frontRightMotor->Set(ControlMode::PercentOutput, 0.0);
	if (m_rearLeftMotor != NULL) m_rearLeftMotor->Set(ControlMode::PercentOutput, 0.0);
	if (m_rearRightMotor != NULL) m_rearRightMotor->Set(ControlMode::PercentOutput, 0.0);
}

#ifdef _Win32

bool RobotControlCommon_ShowControlsDefault()
{
	return false;
}
static std::function<bool(void)> s_ShowControlsCallback = RobotControlCommon_ShowControlsDefault;

void RobotControlCommon_SetShowControlsCallback(std::function<bool(void)> callback)
{
	s_ShowControlsCallback = callback;
}


//Ideally we have only one robot control for both the tester and the main app... we can use this macro in tester to control if we want to see that the calls
//are correctly working.  Typically we shouldn't need to enable this unless there is a problem, or alternatively to verify the actual controls are being sent out
#define __SHOW_SMARTDASHBOARD__

#ifdef __SHOW_SMARTDASHBOARD__
  /***********************************************************************************************************************************/
 /*														Control_1C_Element_UI														*/
/***********************************************************************************************************************************/

Control_1C_Element_UI::Control_1C_Element_UI(uint8_t moduleNumber, uint32_t channel,const char *name,double DefaultNumber) : m_DefaultNumber(DefaultNumber),
	m_PutNumber_Used(false),m_PutBoolUsed(false)
{
	m_Name=name;
	char Buffer[4];
	m_Name+="_";
	itoa(channel,Buffer,10);
	m_Name+=Buffer;
	m_Name+="_";
	itoa(moduleNumber,Buffer,10);
	m_Name+=Buffer;
}

void Control_1C_Element_UI::display_number(double value)
{
	if (s_ShowControlsCallback())
		SmartDashboard::PutNumber(m_Name,value);
}

void Control_1C_Element_UI::display_bool(bool value)
{
	if (s_ShowControlsCallback())
		SmartDashboard::PutBoolean(m_Name,value);
}

//Note: For the get implementation, I restrict use of the bool used members to these functions as a first run
//It's just makes there use-case more restricted

bool Control_1C_Element_UI::get_bool() const
{
	bool ret = false;
	if (s_ShowControlsCallback())
	{
		if (!m_PutBoolUsed)
		{
			SmartDashboard::PutBoolean(m_Name, false);
			m_PutBoolUsed = true;
		}
		ret=SmartDashboard::GetBoolean(m_Name);
	}
	return ret;
}

double Control_1C_Element_UI::get_number() const
{
	double ret = m_DefaultNumber;
	if (s_ShowControlsCallback())
	{
		if (!m_PutNumber_Used)
		{
			SmartDashboard::PutNumber(m_Name, m_DefaultNumber);
			m_PutNumber_Used = true;
		}
		ret=(double)SmartDashboard::GetNumber(m_Name);
	}
	return ret;
}

  /***********************************************************************************************************************************/
 /*														Control_2C_Element_UI														*/
/***********************************************************************************************************************************/

Control_2C_Element_UI::Control_2C_Element_UI(uint8_t moduleNumber, uint32_t forward_channel, uint32_t reverse_channel,const char *name)
{
	m_Name=name;
	char Buffer[4];
	m_Name+="_";
	itoa(forward_channel,Buffer,10);
	m_Name+=Buffer;
	m_Name+="_";
	itoa(reverse_channel,Buffer,10);
	m_Name+=Buffer;
	m_Name+="_";
	itoa(moduleNumber,Buffer,10);
	m_Name+=Buffer;
}

void Control_2C_Element_UI::display_bool(bool value)
{
	if (s_ShowControlsCallback())
		SmartDashboard::PutBoolean(m_Name,value);
}

void Control_2C_Element_UI::display_number(double value)
{
	if (s_ShowControlsCallback())
		SmartDashboard::PutNumber(m_Name,value);
}

bool Control_2C_Element_UI::get_bool(bool defaultvalue) const
{
	bool ret = defaultvalue;
	if (s_ShowControlsCallback())
		ret=SmartDashboard::GetBoolean(m_Name);
	return ret;
}

double Control_2C_Element_UI::get_number() const
{
	double ret = 0.0;
	if (s_ShowControlsCallback())
		ret=SmartDashboard::GetNumber(m_Name);
	return ret;
}


#else  //__SHOW_SMARTDASHBOARD__

  /***********************************************************************************************************************************/
 /*														Control_1C_Element_UI														*/
/***********************************************************************************************************************************/

Control_1C_Element_UI::Control_1C_Element_UI(uint8_t moduleNumber, uint32_t channel,const char *name,double DefaultNumber) : m_DefaultNumber(DefaultNumber),
	m_PutNumber_Used(false),m_PutBoolUsed(false)
{
	m_Name=name;
	char Buffer[4];
	m_Name+="_";
	itoa(channel,Buffer,10);
	m_Name+=Buffer;
	m_Name+="_";
	itoa(moduleNumber,Buffer,10);
	m_Name+=Buffer;
}

void Control_1C_Element_UI::display_number(double value)
{
}

void Control_1C_Element_UI::display_bool(bool value)
{
}

//Note: For the get implementation, I restrict use of the bool used members to these functions as a first run
//It's just makes there use-case more restricted

bool Control_1C_Element_UI::get_bool() const
{
	return false;
}

double Control_1C_Element_UI::get_number() const
{
	return 0.0;
}

  /***********************************************************************************************************************************/
 /*														Control_2C_Element_UI														*/
/***********************************************************************************************************************************/

Control_2C_Element_UI::Control_2C_Element_UI(uint8_t moduleNumber, uint32_t forward_channel, uint32_t reverse_channel,const char *name)
{
	m_Name=name;
	char Buffer[4];
	m_Name+="_";
	itoa(forward_channel,Buffer,10);
	m_Name+=Buffer;
	m_Name+="_";
	itoa(reverse_channel,Buffer,10);
	m_Name+=Buffer;
	m_Name+="_";
	itoa(moduleNumber,Buffer,10);
	m_Name+=Buffer;
}

void Control_2C_Element_UI::display_bool(bool value)
{
}

void Control_2C_Element_UI::display_number(double value)
{
}

bool Control_2C_Element_UI::get_bool() const
{
	return false;
}

double Control_2C_Element_UI::get_number() const
{
	return 0.0;
}

#endif  //__SHOW_SMARTDASHBOARD__


#endif  //_Win32
#endif