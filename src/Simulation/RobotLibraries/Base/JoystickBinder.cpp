#include "Base_Includes.h"
#include "misc.h"
#include "Event.h"
#include "EventMap.h"
#include "Joystick.h"
#include "JoystickBinder.h"

using namespace Framework::UI;
using namespace Framework::Base;
using namespace Framework;

const double DOUBLE_CLICK_TIME = 0.25;

KeyAdd_proto Framework::UI::g_KeybaordSupport_Add=nullptr;
KeyRemove_proto Framework::UI::g_KeybaordSupport_Remove = nullptr;

  /***************************************************************************************************************/
 /*											JoyStick_Binder														*/
/***************************************************************************************************************/

JoyStick_Binder::JoyStick_Binder(IJoystick &joystick) :  m_Joystick(joystick),m_controlledEventMap(NULL), m_eventTime(0.0)
{
	memset(m_lastReleaseTime,0,sizeof(m_lastReleaseTime));
}

JoyStick_Binder::~JoyStick_Binder()
{
	for (JoyAnalogBindings::iterator iter=m_JoyAnalogBindings.begin();iter!=m_JoyAnalogBindings.end();iter++)
	{
		std::vector<std::string> *test=(*iter).second;
		delete test;
		(*iter).second=NULL;
	}
	for (AssignedJoyAnalogs::iterator iter=m_AssignedJoyAnalogs.begin();iter!=m_AssignedJoyAnalogs.end();iter++)
	{
		std::vector<Analog_EventEntry> *test=(*iter).second;
		delete test;
		(*iter).second=NULL;
	}
	for (JoyButtonBindings::iterator iter=m_JoyButtonBindings.begin();iter!=m_JoyButtonBindings.end();iter++)
	{
		std::vector<std::string> *test=(*iter).second;
		delete test;
		(*iter).second=NULL;
	}
	for (AssignedJoyButtons::iterator iter=m_AssignedJoyButtons.begin();iter!=m_AssignedJoyButtons.end();iter++)
	{
		std::vector<Button_EventEntry> *test=(*iter).second;
		delete test;
		(*iter).second=NULL;
	}
}

void JoyStick_Binder::SetControlledEventMap(Framework::Base::EventMap* em)
{
	if (m_controlledEventMap != em)
		m_controlledEventMap = em;
}

void JoyStick_Binder::Add_Analog_Binding_Common(Analog_EventEntry &key,const char eventName[])
{
	std::vector<std::string> *eventNames = m_JoyAnalogBindings[key];
	if (!eventNames)
	{
		eventNames = new std::vector<std::string>;
		eventNames->push_back(eventName);
		m_JoyAnalogBindings[key] = eventNames;
	}
	else
	{
		bool exists = false;
		std::vector<std::string>::iterator pos;
		for (pos = eventNames->begin(); pos != eventNames->end() && !exists; ++pos)
			exists = (eventName == *pos);
		if (!exists)
			eventNames->push_back(eventName);
	}

	std::vector<Analog_EventEntry> *keys = m_AssignedJoyAnalogs[eventName];
	if (!keys)
	{
		keys = new std::vector<Analog_EventEntry>;
		keys->push_back(key);
		m_AssignedJoyAnalogs[eventName] = keys;
	}
	else
	{
		bool exists = false;
		std::vector<Analog_EventEntry>::iterator pos;
		//Check for duplicate entries of the same key (This may be a typical case)
		for (pos = keys->begin(); pos != keys->end() && !exists; ++pos)
			exists = (key == *pos);
		if (!exists)
			keys->push_back(key);
	}
}

void JoyStick_Binder::AddJoy_Analog_Binding(JoyAxis_enum WhichAxis,const char eventName[],bool IsFlipped,double Multiplier,
															  double FilterRange,double CurveIntensity,const char ProductName[])
{
	Analog_EventEntry key(WhichAxis,ProductName,IsFlipped,Multiplier,FilterRange,CurveIntensity);
	Add_Analog_Binding_Common(key,eventName);
}

void JoyStick_Binder::AddJoy_Culver_Binding(JoyAxis_enum WhichXAxis,JoyAxis_enum WhichYAxis,double MagnitudeScalarArc,double MagnitudeScalarBase,const char eventName[],bool IsFlipped,double Multiplier,
											double FilterRange,double CurveIntensity,const char ProductName[])
{
	Culver_EventEntry key(WhichXAxis,WhichYAxis,MagnitudeScalarArc,MagnitudeScalarBase,ProductName,IsFlipped,Multiplier,FilterRange,CurveIntensity);
	Add_Analog_Binding_Common(key,eventName);
}

void JoyStick_Binder::AddJoy_SplitAxis_Binding(JoyAxis_enum Which1Axis,JoyAxis_enum Which2Axis,bool CenterPointIdle,const char eventName[],bool IsFlipped,double Multiplier,
											double FilterRange,double CurveIntensity,const char ProductName[])
{
	SplitAxis_EventEntry key(Which1Axis,Which2Axis,CenterPointIdle,ProductName,IsFlipped,Multiplier,FilterRange,CurveIntensity);
	Add_Analog_Binding_Common(key,eventName);
}

void JoyStick_Binder::RemoveJoy_Analog_Binding(const char eventName[],const char ProductName[])
{
	AssignedJoyAnalogs::iterator iter;
	iter=m_AssignedJoyAnalogs.find(eventName);
	if (iter!=m_AssignedJoyAnalogs.end())
	{
		std::vector<Analog_EventEntry> *keys = (*iter).second;

		std::vector<Analog_EventEntry>::iterator pos;
		//Check for duplicate entries of the same key (This may be a typical case)
		for (pos = keys->begin(); pos != keys->end(); ++pos)
		{
			//find matching product
			if (strcmp((*pos).ProductName.c_str(),ProductName)==0)
			{
				//Now to clean out the joy analog binding
				{
					JoyAnalogBindings::iterator b_iter=m_JoyAnalogBindings.find(*pos);
					assert (b_iter!=m_JoyAnalogBindings.end());

					std::vector<std::string> *eventNames = (*b_iter).second;
					std::vector<std::string>::iterator pos;
					for (pos = eventNames->begin(); pos != eventNames->end(); ++pos)
					{
						if (strcmp((*pos).c_str(),eventName)==0)
						{
							pos=eventNames->erase(pos);
							//Note: it should only be one entry... but we'll let it go just in case its more
							if (pos == eventNames->end())
								break;
						}
					}
					if (eventNames->empty())
						m_JoyAnalogBindings.erase(b_iter);
				}
				pos=keys->erase(pos);
				//Note: it should only be one entry... but we'll let it go just in case its more
				if (pos == keys->end())
					break;
			}
		}
		//if list is empty delete event entry as well
		if (keys->empty())
			m_AssignedJoyAnalogs.erase(iter);
	}
}

void JoyStick_Binder::AddJoy_Button_Binding(size_t WhichButton,const char eventName[],bool useOnOff,bool dbl_click,const char ProductName[])
{
	Button_EventEntry key(WhichButton,ProductName,useOnOff,dbl_click);
	std::vector<std::string>* eventNames = m_JoyButtonBindings[key];
	if (!eventNames)
	{
		eventNames = new std::vector<std::string>;
		eventNames->push_back(eventName);
		m_JoyButtonBindings[key] = eventNames;
	}
	else
	{
		bool exists = false;
		std::vector<std::string>::iterator pos;
		for (pos = eventNames->begin(); pos != eventNames->end() && !exists; ++pos)
			exists = (eventName == *pos);
		if (!exists)
			eventNames->push_back(eventName);
	}

	std::vector<Button_EventEntry> *keys = m_AssignedJoyButtons[eventName];
	if (!keys)
	{
		keys = new std::vector<Button_EventEntry>;
		keys->push_back(key);
		m_AssignedJoyButtons[eventName] = keys;
	}
	else
	{
		bool exists = false;
		std::vector<Button_EventEntry>::iterator pos;
		//Check for duplicate entries of the same key (This may be a typical case)
		for (pos = keys->begin(); pos != keys->end() && !exists; ++pos)
			exists = (key == *pos);
		if (!exists)
			keys->push_back(key);
	}
}

void JoyStick_Binder::RemoveJoy_Button_Binding(const char eventName[],const char ProductName[])
{
	AssignedJoyButtons::iterator iter;
	iter=m_AssignedJoyButtons.find(eventName);
	if (iter!=m_AssignedJoyButtons.end())
	{
		std::vector<Button_EventEntry> *keys = (*iter).second;

		std::vector<Button_EventEntry>::iterator pos;
		//Check for duplicate entries of the same key (This may be a typical case)
		for (pos = keys->begin(); pos != keys->end(); ++pos)
		{
			//find matching product
			if (strcmp((*pos).ProductName.c_str(),ProductName)==0)
			{
				//Now to clean out the joy button binding
				{
					JoyButtonBindings::iterator b_iter=m_JoyButtonBindings.find(*pos);
					assert (b_iter!=m_JoyButtonBindings.end());

					std::vector<std::string> *eventNames = (*b_iter).second;
					std::vector<std::string>::iterator pos;
					for (pos = eventNames->begin(); pos != eventNames->end(); ++pos)
					{
						if (strcmp((*pos).c_str(),eventName)==0)
						{
							pos=eventNames->erase(pos);
							//Note: it should only be one entry... but we'll let it go just in case its more
							if (pos == eventNames->end())
								break;
						}
					}
					if (eventNames->empty())
						m_JoyButtonBindings.erase(b_iter);
				}
				pos=keys->erase(pos);
				//Note: it should only be one entry... but we'll let it go just in case its more
				if (pos == keys->end())
					break;
			}
		}
		//if list is empty delete event entry as well
		if (keys->empty())
			m_AssignedJoyButtons.erase(iter);
	}
}

void JoyStick_Binder::AddJoy_Analog_Default(JoyAxis_enum WhichAxis,const char eventName[],bool IsFlipped,double Multiplier,double FilterRange,double CurveIntensity,const char ProductName[])
{
	//removed intercept since we are not using a config manager
		AddJoy_Analog_Binding(WhichAxis,eventName,IsFlipped,Multiplier,FilterRange,CurveIntensity,ProductName);
}

void JoyStick_Binder::AddJoy_Culver_Default(JoyAxis_enum WhichXAxis,JoyAxis_enum WhichYAxis,double MagnitudeScalarArc,double MagnitudeScalarBase,const char eventName[],bool IsFlipped,
											double Multiplier,double FilterRange,double CurveIntensity,const char ProductName[])
{
	//removed intercept since we are not using a config manager
		AddJoy_Culver_Binding(WhichXAxis,WhichYAxis,MagnitudeScalarArc,MagnitudeScalarBase,eventName,IsFlipped,Multiplier,FilterRange,CurveIntensity,ProductName);
}

void JoyStick_Binder::AddJoy_SplitAxis_Default(JoyAxis_enum Which1Axis,JoyAxis_enum Which2Axis,bool CenterPointIdle,const char eventName[],bool IsFlipped,double Multiplier,double FilterRange,double CurveIntensity,const char ProductName[])
{
	//removed intercept since we are not using a config manager
		AddJoy_SplitAxis_Binding(Which1Axis,Which2Axis,CenterPointIdle,eventName,IsFlipped,Multiplier,FilterRange,CurveIntensity,ProductName);
}

void JoyStick_Binder::AddJoy_Button_Default(size_t WhichButton,const char eventName[],bool useOnOff,bool dbl_click,const char ProductName[])
{
	//removed intercept since we are not using a config manager
		AddJoy_Button_Binding(WhichButton,eventName,useOnOff,dbl_click,ProductName);
}

bool JoyStick_Binder::IsDoubleClicked(size_t i)
{
	return (m_eventTime-m_lastReleaseTime[i] < DOUBLE_CLICK_TIME);
}

double GetJoystickValue(Framework::Base::IJoystick::JoyState joyinfo,JoyStick_Binder::JoyAxis_enum whichaxis)
{
	double Value=0.0;
	switch(whichaxis)
	{
	case JoyStick_Binder::eX_Axis:	Value=joyinfo.lX;	break;
	case JoyStick_Binder::eY_Axis:	Value=joyinfo.lY;	break;
	case JoyStick_Binder::eZ_Axis:	Value=joyinfo.lZ;	break;
	case JoyStick_Binder::eX_Rot:	Value=joyinfo.lRx;	break;
	case JoyStick_Binder::eY_Rot:	Value=joyinfo.lRy;	break;
	case JoyStick_Binder::eZ_Rot:	Value=joyinfo.lRz;	break;
	case JoyStick_Binder::eSlider0:	Value=joyinfo.rgSlider[0];	break;
	case JoyStick_Binder::eSlider1:	Value=joyinfo.rgSlider[1];	break;
	case JoyStick_Binder::ePOV_0:	Value=joyinfo.rgPOV[0];		break;
	case JoyStick_Binder::ePOV_1:	Value=joyinfo.rgPOV[1];		break;
	case JoyStick_Binder::ePOV_2:	Value=joyinfo.rgPOV[2];		break;
	case JoyStick_Binder::ePOV_3:	Value=joyinfo.rgPOV[3];		break;
	};
	return Value;
}

inline double JoyStick_Binder::AnalogConversionNormal(double InValue,const Analog_EventEntry &key)
{
	double ValueABS=fabs(InValue); //take out the sign... put it back in the end

	//Now to use the attributes to tweak the value
	//First evaluate dead zone range... if out of range subtract out the offset for no loss in precision
	//The /(1.0-filter range) will restore the full range

	ValueABS=(ValueABS>=key.FilterRange) ? ValueABS-key.FilterRange:0.0; 

	ValueABS=key.Multiplier*(ValueABS/(1.0-key.FilterRange)); //apply scale first then
	if (key.CurveIntensity<=1.0)
		ValueABS=key.CurveIntensity*pow(ValueABS,3) + (1.0-key.CurveIntensity)*ValueABS; //apply the curve intensity
	else
		ValueABS=pow(ValueABS,key.CurveIntensity); //apply the curve intensity

	//Now to restore the sign
	const double OutValue=(InValue<0.0)?-ValueABS:ValueABS;
	return OutValue;
}

void JoyStick_Binder::UpdateJoyStick(double dTick_s)
{
	if (!m_controlledEventMap)
		return;

	m_eventTime += dTick_s;

	for (size_t JoyNum=0;JoyNum<m_Joystick.GetNoJoysticksFound();JoyNum++)
	{
		//Poll this Joystick
		Base::IJoystick::JoyState joyinfo;
		const Base::IJoystick::JoystickInfo &joyinfo2=m_Joystick.GetJoyInfo(JoyNum);
		if (m_Joystick.read_joystick(JoyNum,joyinfo))
		{
			//Ensure we have a flood control entry now
			while (m_FloodControl.size()<=JoyNum)
				m_FloodControl.push_back(joyinfo);

			Base::IJoystick::JoyState &floodcontrol=m_FloodControl[JoyNum];
			//here's a quick test to see if we are working
			//printf("\r no=%d xaxis=%f yaxis=%f  button=0x%x             ",joyinfo.JoystickNumber,joyinfo.lX,joyinfo.lY,joyinfo.ButtonBank[0]);

			//Now to iterate each slider and button to fire events for them
			//first the sliders
			for (size_t i=0;i<eNoJoyAxis_Entries;i++)
			{
				double Value,OldValue;
				//This assumes that JoyAxis_enum and JoyCapFlag_enum have integrity... a bit fragile assumption but effective for performance
				bool IsSupported=((1<<i)&joyinfo2.JoyCapFlags)!=0;
				//Only fire events for axis that the joystick supports
				if (IsSupported)
				{
					Value=GetJoystickValue(joyinfo,(JoyAxis_enum)i);
					OldValue=GetJoystickValue(floodcontrol,(JoyAxis_enum)i);

					//Note: at this time I cannot perform flood control for axis controls because of the nature of how
					//the client code needs a constant update on the position (as it will zero out afterward)
					//For now we'll abide by these requirements, but I may want to evaluate that and see if we can 
					//support flood control for this
					//We need not worry about a false positive we can rest assured they will be equal if they are
					//if (Value!=OldValue)
					{
						//I'm writing this out long hand so that I can access my original key (grabbing its contents)
						//std::vector<std::string> *AnalogEvents=GetBindingsForJoyAnalog((JoyAxis_enum)i);

						JoyAnalogBindings::iterator iter;
						{
							//First try to find the product specific binding
							Analog_EventEntry key((JoyAxis_enum)i,joyinfo2.ProductName.c_str());
							iter=m_JoyAnalogBindings.find(key);
						}

						//If the product is not found (typical) try to find any
						if (iter==m_JoyAnalogBindings.end())
							iter=m_JoyAnalogBindings.find((JoyAxis_enum)i);

						//If we found any events for this axis
						if (iter!=m_JoyAnalogBindings.end())
						{
							std::vector<std::string> *AnalogEvents=(*iter).second;
							const Analog_EventEntry &key=(*iter).first;

							if (AnalogEvents)
							{

								switch (key.AnalogEntryType)
								{
								case Analog_EventEntry::eAnalog_EventEntryType_Culver:
									{
									double ValueABS=fabs(Value); //take out the sign... put it back in the end
									double YValue=GetJoystickValue(joyinfo,key.ExtraData.culver.WhichYAxis);
									//Find arc tangent of wheel stick relative to vertical... note relative to vertical suggests that we swap the x and y components!
									//NOTE if we want to extend past 90 use this:   we could have logic to support it
									//const double theta = atan2(Value,-YValue);
									//This version limits to 90 degrees... so any down motion will be treated like up motion
									const double theta = atan2(Value,fabs(YValue));
									//the magnitude scalar is a blend of arc and base constants depending on the values current position
									const double magnitude_scalar=((1.0-ValueABS) * key.ExtraData.culver.MagnitudeScalarArc)+(ValueABS * key.ExtraData.culver.MagnitudeScalarBase);
									//Find the magnitude of the wheel stick
									const double magnitude = sqrt(((Value * Value) + (YValue * YValue))) * magnitude_scalar;
									//DOUT4("%.2f,%.2f,%f,%f",RAD_2_DEG(theta),magnitude,Value,theta*magnitude);
									//Assign the new value
									Value=theta*magnitude;   //note theta holds the sign
									//Now to apply the other conversions to this
									Value=AnalogConversionNormal(Value,key);
									}
									break;
								case Analog_EventEntry::eAnalog_EventEntryType_SplitAxis:
									{
										const double Joy2Value=GetJoystickValue(joyinfo,key.ExtraData.split_axis.Which2Axis);
										double Value2=AnalogConversionNormal(Joy2Value,key);
										//invert the side of the negative where by default this will put the highest value at the end
										double Value1=-AnalogConversionNormal(Value,key);
										if (key.ExtraData.split_axis.CenterPointIdle)
										{
											//normalize to a 0..1 range
											Value2=fabs(Value2);
											Value1=-fabs(Value1);
										}
										else
										{
											//normalize to a 0..1 range
											Value2=(Value2+1.0)/2.0;
											Value1=(Value1+1.0)/2.0;
											//Now to put Value1 on the negative range number line
											Value1-=1.0;
										}
										//Now we can add them for the final value;
										Value=Value1+Value2;
									}
									break;
								case Analog_EventEntry::eAnalog_EventEntryType_Normal:
									Value=AnalogConversionNormal(Value,key);
									break;
								}
	
								std::vector<std::string>::iterator pos;
								for (pos = AnalogEvents->begin(); pos != AnalogEvents->end(); ++pos)
									m_controlledEventMap->EventValue_Map[*pos].Fire(key.IsFlipped?-Value:Value);
							}
						}
					}
				}
			}
			//Now the buttons
			for (size_t i=0;i<32;i++)
			{
				//Flood control check
				if ((joyinfo.ButtonBank[0] ^ floodcontrol.ButtonBank[0]) & (1<<i))
				{
					bool IsPressed=((1<<i) & joyinfo.ButtonBank[0])!=0;

					//I'm writing this out long hand so that I can access my original key (grabbing its contents)
					//std::vector<std::string> *ButtonEvents=GetBindingsForJoyButton(i);

					JoyButtonBindings::iterator iter;
					if (!IsPressed) m_lastReleaseTime[i]=m_eventTime;
					{
						//Since it is more expensive to find elements than to test dbl click we'll go ahead and test every case
						//The worst case scenario is nudging for keys that have no double click support which costs an additional
						//2 finds that fail.  Most of the time nudges are rare, and a double click initiated has no penalty
						if (IsPressed && IsDoubleClicked(i))
						{
							//First try to find the product specific binding
							Button_EventEntry key(i,joyinfo2.ProductName.c_str(),true,true);
							iter=m_JoyButtonBindings.find(key);
							//If the product is not found (typical) try to find any
							if (iter==m_JoyButtonBindings.end())
								iter=m_JoyButtonBindings.find(Button_EventEntry(i,"any",true,true));
							//See if there are any double clicked events found
							if (iter!=m_JoyButtonBindings.end())
								m_UseDoubleClickBindings[i]=iter;
						}
					}

					//TODO see if there is a more proper std method to test for NULL (e.g. empty, which doesn't work here)
					//if (!m_UseDoubleClickBindings[i]._Mynode())
					if (true)
					{
						//First try to find the product specific binding
						Button_EventEntry key(i,joyinfo2.ProductName.c_str());
						iter=m_JoyButtonBindings.find(key);

						//If the product is not found (typical) try to find any
						if (iter==m_JoyButtonBindings.end())
							iter=m_JoyButtonBindings.find(i);
					}
					else
						iter=m_UseDoubleClickBindings[i];  //for release case (arguably would check for release here)

					//If we found any events for this button
					if (iter!=m_JoyButtonBindings.end())
					{
						std::vector<std::string> *ButtonEvents=(*iter).second;
						const Button_EventEntry &key=(*iter).first;

						if (!IsPressed)
							m_UseDoubleClickBindings[i]=JoyButtonBindings::iterator();  //ensure double click setting is reset for release

						if (ButtonEvents)
						{
							std::vector<std::string>::iterator pos;
							//fire the correct event based on the useOnOff attribute
							if (key.useOnOff)
							{
								for (pos = ButtonEvents->begin(); pos != ButtonEvents->end(); ++pos)
									m_controlledEventMap->EventOnOff_Map[*pos].Fire(IsPressed);
							}
							else
							{
								//Send on the down key only
								if (IsPressed)
								{
									for (pos = ButtonEvents->begin(); pos != ButtonEvents->end(); ++pos)
										m_controlledEventMap->Event_Map[*pos].Fire();
								}
							}
						}
					}
				}
			}
			//Update flood control
			floodcontrol=joyinfo;
		}
	}
}


