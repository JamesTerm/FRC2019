#pragma once
#include <functional>
namespace Framework
{
	namespace UI
	{

//By default set to nullptr since robots don't receive keyboard input from the driver station (yet)
//It is possible to use the network tables to receive them if we want
using KeyAdd_proto = std::function<bool(Framework::Base::Key key, const std::string eventName, bool useOnOff, bool ForceBindThisKey)>;
extern KeyAdd_proto g_KeybaordSupport_Add;
using KeyRemove_proto = std::function<void(Framework::Base::Key key, const std::string eventName, bool useOnOff)>;
extern KeyRemove_proto g_KeybaordSupport_Remove;

class KeyboardMouse_CB
{
public:
	KeyboardMouse_CB() {}
	bool AddKeyBinding(Framework::Base::Key key, const std::string eventName, bool useOnOff, bool ForceBindThisKey = false)
	{
		bool ret = false;
		if (g_KeybaordSupport_Add)
			ret=g_KeybaordSupport_Add(key, eventName, useOnOff, ForceBindThisKey);
		else
			ret=false;
		return ret;
	}
	void RemoveKeyBinding(Framework::Base::Key key, std::string eventName, bool useOnOff) 
	{
		if (g_KeybaordSupport_Remove)
			g_KeybaordSupport_Remove(key, eventName, useOnOff);
	}

	/// This version of the function is just for easy porting from the old KB technique
	void AddKeyBindingR(bool useOnOff, std::string eventName, Framework::Base::Key key)
		{AddKeyBinding(key, eventName, useOnOff);}
	static void SetKeyboardSupport_Add(KeyAdd_proto callback) 
		{	g_KeybaordSupport_Add = callback; 
		}
	static void SetKeyboardSupport_Remove(KeyRemove_proto callback) { g_KeybaordSupport_Remove = callback; }
};

class JoyStick_Binder
{
public:
	//grant access to the Joystick interface to obtain info like number of joysticks and capabilities
	Framework::Base::IJoystick &GetJoystick() const;

	enum JoyAxis_enum
	{
		eX_Axis,
		eY_Axis,
		eZ_Axis,
		eX_Rot,
		eY_Rot,
		eZ_Rot,
		eSlider0,
		eSlider1,
		ePOV_0,
		ePOV_1,
		ePOV_2,
		ePOV_3,
		eNoJoyAxis_Entries
	};

	JoyStick_Binder(Framework::Base::IJoystick &joystick);
	~JoyStick_Binder();
	///This binds all the axis, rotations, sliders to an event
	/// \param IsFlipped this will simply multiply a -1.0 coefficient
	void AddJoy_Analog_Default(JoyAxis_enum WhichAxis,const char eventName[],bool IsFlipped=false,double Multiplier=1.0,double FilterRange=0.0,
		double CurveIntensity=0.0,const char ProductName[]="any");
	void AddJoy_Culver_Default(JoyAxis_enum WhichXAxis,JoyAxis_enum WhichYAxis,double MagnitudeScalar_arc,double MagnitudeScalar_base,const char eventName[],bool IsFlipped=false,double Multiplier=1.0,double FilterRange=0.0,
		double CurveIntensity=0.0,const char ProductName[]="any");
	void AddJoy_SplitAxis_Default(JoyAxis_enum Which1Axis,JoyAxis_enum Which2Axis,bool CenterPointIdle,const char eventName[],bool IsFlipped=false,double Multiplier=1.0,double FilterRange=0.0,
		double CurveIntensity=0.0,const char ProductName[]="any");
	/// \param WhichButton while in theory there are up to 128 buttons supported I'm only going to support the first 32 for now
	/// Use the JoystickTest program to determine the numbers of the buttons
	void AddJoy_Button_Default(size_t WhichButton,const char eventName[],bool useOnOff=true,bool dbl_click=false,const char ProductName[]="any");
	//TODO see if I really need RemoveJoy_x_Binding methods


	// This is here because producer's update is not virtual
	void UpdateJoyStick(double dTick_s);

	//TODO get with Rick to see if I need to fire events like keyboard for this mutator.  I know that a simple assignment is adequate for the current
	//stresses presented 
	//  [2/5/2010 JamesK]
	void SetControlledEventMap(Framework::Base::EventMap* em);
	//Event3<JoyStick_Binder*, GG_Framework::UI::EventMap*, GG_Framework::UI::EventMap*> EventMapChanged; //!< <this, old, new> fired before change

	void RemoveJoy_Analog_Binding(const char eventName[],const char ProductName[]="any");
	void RemoveJoy_Button_Binding(const char eventName[],const char ProductName[]="any");

private:

	void AddJoy_Analog_Binding(JoyAxis_enum WhichAxis,const char eventName[],bool IsFlipped=false,double Multiplier=1.0,double FilterRange=0.0,
		double CurveIntensity=0.0,const char ProductName[]="any");
	void AddJoy_Culver_Binding(JoyAxis_enum WhichXAxis,JoyAxis_enum WhichYAxis,double MagnitudeScalar_arc,double MagnitudeScalar_base,const char eventName[],bool IsFlipped=false,double Multiplier=1.0,double FilterRange=0.0,
		double CurveIntensity=0.0,const char ProductName[]="any");
	void AddJoy_SplitAxis_Binding(JoyAxis_enum Which1Axis,JoyAxis_enum Which2Axis,bool CenterPointIdle,const char eventName[],bool IsFlipped=false,double Multiplier=1.0,double FilterRange=0.0,
		double CurveIntensity=0.0,const char ProductName[]="any");
	void AddJoy_Button_Binding(size_t WhichButton,const char eventName[],bool useOnOff=true,bool dbl_click=false,const char ProductName[]="any");

	struct EventEntry_Base
	{
		EventEntry_Base(const char _ProductName[]) : ProductName(_ProductName) {}
		std::string ProductName;	//Default is "any"    Use Joystick test to find this if you configure multiple joysticks
		//std::string InstanceName;	//Default is "any"    Use Joystick test to find this if you configure multiple joysticks
	};
	//TODO support instance name for the > = operators, and allow any to pass test
	struct Analog_EventEntry : public EventEntry_Base
	{
		enum Analog_EventEntryType
		{
			eAnalog_EventEntryType_Normal,
			eAnalog_EventEntryType_Culver,
			eAnalog_EventEntryType_SplitAxis
		};
		Analog_EventEntry(JoyAxis_enum _WhichAxis,const char _ProductName[]="any",bool _IsFlipped=false,double _Multiplier=1.0,
			double _FilterRange=0.0,double _CurveIntensity=false,Analog_EventEntryType _AnalogType=eAnalog_EventEntryType_Normal) : 
		EventEntry_Base(_ProductName),AnalogEntryType(_AnalogType),WhichAxis(_WhichAxis),Multiplier(_Multiplier),
			FilterRange(_FilterRange),CurveIntensity(_CurveIntensity),IsFlipped(_IsFlipped)
		{
			//Init the extra data for debugging purposes (to avoid seeing garbage)
			if (_AnalogType==eAnalog_EventEntryType_Normal)
				ExtraData.raw=0;
		}

		Analog_EventEntryType AnalogEntryType;
		JoyAxis_enum WhichAxis;
		double Multiplier;
		double FilterRange;
		double CurveIntensity;
		bool IsFlipped;
		//This allows extra space needed for derived types
		union uSpecificData
		{
			size_t raw;
			struct CulverData  //data used in Culver entries
			{
				JoyAxis_enum WhichYAxis;  
				double MagnitudeScalarArc,MagnitudeScalarBase;
			} culver;
			struct SplitAxis  //data used in Split Axis entries
			{
				JoyAxis_enum Which2Axis;
				bool CenterPointIdle;   //some axis idle at 0, while some idle full range
			} split_axis;
		} ExtraData;
		bool operator >  (const Analog_EventEntry& rhs) const { return ((WhichAxis == rhs.WhichAxis) ? (ProductName > rhs.ProductName) : (WhichAxis > rhs.WhichAxis)); }
		bool operator == (const Analog_EventEntry& rhs) const { return (WhichAxis == rhs.WhichAxis) && (ProductName == rhs.ProductName); }
	};
	//The Culver entry is like the analog entry except it needs two axis readings to work as one
	struct Culver_EventEntry : public Analog_EventEntry
	{
		Culver_EventEntry(JoyAxis_enum _WhichXAxis,JoyAxis_enum _WhichYAxis,double MagnitudeScalarArc,double MagnitudeScalarBase,const char _ProductName[]="any",bool _IsFlipped=false,double _Multiplier=1.0,
			double _FilterRange=0.0,double _CurveIntensity=false) : 
		Analog_EventEntry(_WhichXAxis,_ProductName,_IsFlipped,_Multiplier,_FilterRange,_CurveIntensity,eAnalog_EventEntryType_Culver)
		{
			ExtraData.culver.WhichYAxis=_WhichYAxis;
			ExtraData.culver.MagnitudeScalarArc=MagnitudeScalarArc;
			ExtraData.culver.MagnitudeScalarBase=MagnitudeScalarBase;
		}
	};
	struct SplitAxis_EventEntry : public Analog_EventEntry
	{
		SplitAxis_EventEntry(JoyAxis_enum _Which1Axis,JoyAxis_enum _Which2Axis,bool _CenterPointIdle=true,const char _ProductName[]="any",bool _IsFlipped=false,double _Multiplier=1.0,
			double _FilterRange=0.0,double _CurveIntensity=false) : 
		Analog_EventEntry(_Which1Axis,_ProductName,_IsFlipped,_Multiplier,_FilterRange,_CurveIntensity,eAnalog_EventEntryType_SplitAxis)
		{
			ExtraData.split_axis.Which2Axis=_Which2Axis;
			ExtraData.split_axis.CenterPointIdle=_CenterPointIdle;
		}
	};

	struct Button_EventEntry : public EventEntry_Base
	{
		Button_EventEntry(size_t _WhichButton,const char _ProductName[]="any",bool _useOnOff=true,bool _dbl_click=false) : 
	EventEntry_Base(_ProductName),WhichButton(_WhichButton),useOnOff(_useOnOff),dbl_click(_dbl_click)
	{}

	size_t WhichButton;
	bool useOnOff;
	bool dbl_click;
	bool operator >  (const Button_EventEntry& rhs) const { return ((WhichButton == rhs.WhichButton) ? 
		ProductName==rhs.ProductName ? 
		(dbl_click > rhs.dbl_click) : (ProductName > rhs.ProductName) : 
	(WhichButton > rhs.WhichButton)); }
	bool operator == (const Button_EventEntry& rhs) const { return (WhichButton == rhs.WhichButton) && (ProductName == rhs.ProductName) && (dbl_click == rhs.dbl_click); }
	};

	inline double AnalogConversionNormal(double InValue,const Analog_EventEntry &key);
	void Add_Analog_Binding_Common(Analog_EventEntry &key,const char eventName[]);

	std::vector<std::string> *GetBindingsForJoyAnalog(Analog_EventEntry EventEntry) {return m_JoyAnalogBindings[EventEntry];}
	std::vector<std::string> *GetBindingsForJoyButton(Button_EventEntry EventEntry) {return m_JoyButtonBindings[EventEntry];}
	bool IsDoubleClicked(size_t i);

	typedef std::map<std::string, std::vector<Analog_EventEntry>*, std::greater<std::string> > AssignedJoyAnalogs;
	typedef std::map<std::string, std::vector<Button_EventEntry>*, std::greater<std::string> > AssignedJoyButtons;
	typedef std::map<Analog_EventEntry, std::vector<std::string>*, std::greater<Analog_EventEntry> > JoyAnalogBindings;
	typedef std::map<Button_EventEntry, std::vector<std::string>*, std::greater<Button_EventEntry> > JoyButtonBindings;
	JoyAnalogBindings m_JoyAnalogBindings;
	AssignedJoyAnalogs m_AssignedJoyAnalogs;
	JoyButtonBindings m_JoyButtonBindings;
	AssignedJoyButtons m_AssignedJoyButtons;

	Framework::Base::IJoystick &m_Joystick;
	Framework::Base::EventMap *m_controlledEventMap;
	std::vector<Framework::Base::IJoystick::JoyState> m_FloodControl;
	//This will measure each buttons release time!
	double m_lastReleaseTime[32];
	JoyButtonBindings::iterator m_UseDoubleClickBindings[32]; //These are in essence pointers
	double m_eventTime;
};

	}
}

namespace UI=Framework::UI;
