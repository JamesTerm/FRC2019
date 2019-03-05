#pragma once
typedef unsigned short	USHORT;
typedef	unsigned char	UINT8;
typedef	unsigned int	UINT32;

namespace frc
{
#ifndef _Win32
///This class is an Encoder that provides a more accurate way to obtain the current rate
class Encoder2 : public Encoder
{
	public:
		//depreciated in favor of having a similar interface with solenoids
		//Encoder2(UINT32 aChannel, UINT32 bChannel, bool reverseDirection=false, CounterBase::EncodingType encodingType = k4X);
		Encoder2(UINT8 ModuleNumber, UINT32 aChannel, UINT32 bChannel, bool reverseDirection=false, CounterBase::EncodingType encodingType = k4X);

		///This uses a different technique of obtaining the rate by use of the actual pulse count.  This should produce less noisy results
		///at higher speeds.  This is simple as it does not support overlap as it would take about 5 continuous 
		///hours at 78 rps before the end is reached
		/// \param dTime_s delta time slice in seconds
		double GetRate2(double dTime_s);
		///unfortunately reset is not virtual... client code should call this if using reset
		void Reset2();
	private:
		double m_LastDistance;  //keep note of last distance
};
#endif

class Driver_Station_Joystick : public Framework::Base::IJoystick
{	
	public:
		typedef LUA_Controls_Properties::DriverStation_Slot_List Driver_Station_SlotList;  //we'll just make our own type, but we could include ship.h

		//Note: this current configuration requires the two joysticks reside in adjacent ports (e.g. 2,3)
		Driver_Station_Joystick(int NoJoysticks,int StartingPort);
		virtual ~Driver_Station_Joystick();
		void SetSlotList(const LUA_Controls_Properties &control_props);
	protected:  //from IJoystick
		virtual size_t GetNoJoysticksFound();
		virtual bool read_joystick (size_t nr, JoyState &Info);
		
		virtual const JoystickInfo &GetJoyInfo(size_t nr) const {return m_JoyInfo[nr];}
	private:
		std::vector<JoystickInfo> m_JoyInfo;
		Driver_Station_SlotList m_SlotList;
		DriverStation *m_ds;
		int m_NoJoysticks,m_StartingPort;
};
}