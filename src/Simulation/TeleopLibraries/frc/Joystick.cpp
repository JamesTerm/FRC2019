
#include <frc/WPILib.h>
#include "Joystick.h"
#define FRAMEWORK_BASE_API
#include "DirectInput_Joystick.h"

using namespace frc;

constexpr double kPi = 3.14159265358979323846;



void PollJoyInfo();
static GG_Framework::Base::IJoystick *s_joy=nullptr;  
DirectJoystick_Interface& DirectJoystick_Interface::GetInstance()
{
	static DirectJoystick_Interface s_DJI;
	s_joy = &GG_Framework::Base::GetDirectInputJoystick();
	PollJoyInfo(); //initial poll
	return s_DJI;
}


  /*******************************************************************************************************************************/
 /*													DirectJoystick_Interface													*/
/*******************************************************************************************************************************/

static GG_Framework::Base::IJoystick::JoyState s_joy_state[3];
static GG_Framework::Base::IJoystick::JoystickInfo s_joy_info[3];

void PollJoyInfo()
{
	size_t joy_count = s_joy->GetNoJoysticksFound();
	assert(joy_count < 4);
	for (size_t i = 0; i < joy_count; i++)
	{
		s_joy_info[i] = s_joy->GetJoyInfo(i);
	}
}


bool DirectJoystick_Interface::GetStickButton(int stick, int button) 
{
	assert(button < 32);
	//and a shift bit against the current button bank state to see if its pressed
	int test = s_joy_state[stick].ButtonBank[0] & (1 << button);
	return test!=0; 
}
bool DirectJoystick_Interface::GetStickButtonPressed(int stick, int button) 
{
	return false; 
}
bool DirectJoystick_Interface::GetStickButtonReleased(int stick, int button) 
{ 
	return false; 
}
//Keep things on one thread... we'll issue the read here
double DirectJoystick_Interface::GetStickAxis(int stick, int axis) 
{
	assert(stick < 3);
	//It is possible that I make calls to joysticks that are not connected... check for this
	if (stick >= s_joy->GetNoJoysticksFound())
		return 0.0;
	s_joy->read_joystick(stick, s_joy_state[stick]);
	double ret = 0.0;
	switch (axis)
	{
	case 0:	ret = s_joy_state[stick].lX; break;
	case 1:	ret = s_joy_state[stick].lY; break;
	case 2:	ret = s_joy_state[stick].lZ; break;
	case 3:	ret = s_joy_state[stick].lRx; break;
	case 4:	ret = s_joy_state[stick].lRy; break;
	case 5:	ret = s_joy_state[stick].lRz; break;
	default:
		assert(false);  //what axis is this?
	}
	return ret; 
}
int DirectJoystick_Interface::GetStickPOV(int stick, int pov) 
{ 
	assert(pov < 4);
	return (int)s_joy_state[stick].rgPOV[pov]; 
}
int DirectJoystick_Interface::GetStickButtons(int stick) const 
{
	//Not sure about this one
	return s_joy_state[stick].ButtonBank[0];
}
int DirectJoystick_Interface::GetStickAxisCount(int stick) const 
{ 
	size_t flags = s_joy_info[stick].JoyCapFlags  & 0x3F; //mask for only the first 6 bits
	int count = 0;
	//Gah
	int BitCount_LUT[64] =
	{ 
		0,1,1,2,1,2,2,3, 1,2,2,3,2,3,3,4,   //0-F
		1,2,2,3,2,3,3,4, 2,3,3,4,3,4,4,5,   //0001 10-1F
		1,2,2,3,2,3,3,4, 2,3,3,4,3,4,4,5,   //0010 20-2F
		2,3,3,4,3,4,4,5, 3,4,4,5,4,5,5,6,   //0011 30-3F
	};

	return BitCount_LUT[flags];
}
int DirectJoystick_Interface::GetStickPOVCount(int stick) const 
{ 
	int count = s_joy_info[stick].nPOVCount;
	assert(count < 4);
	return count; 
}
int DirectJoystick_Interface::GetStickButtonCount(int stick) const 
{ 
	int count = s_joy_info[stick].nButtonCount;
	assert(count < 32);
	return count;
}
bool DirectJoystick_Interface::GetJoystickIsXbox(int stick) const 
{ 
	return false;  //direct input doesn't support this
}
int DirectJoystick_Interface::GetJoystickType(int stick) const 
{ 
	return GenericHID::kHIDGamepad;  //doesn't support this either
}
std::string DirectJoystick_Interface::GetJoystickName(int stick) const 
{ 
	return s_joy_info->InstanceName;  //instance may be preferred when having multiple of the same
}
int DirectJoystick_Interface::GetJoystickAxisType(int stick, int axis) const 
{ 
	//currently I'm not remapping, but if I do this will need to be fixed
	return axis; 
}


int32_t frc::HAL_SetJoystickOutputs(int32_t joystickNum, int64_t outputs,
	int32_t leftRumble, int32_t rightRumble) {
	//SimDriverStationData->SetJoystickOutputs(joystickNum, outputs, leftRumble,
	//	rightRumble);
	return 0;
}

  /*******************************************************************************************************************************/
 /*															GenericHID															*/
/*******************************************************************************************************************************/
//GenericHID::GenericHID(int port) : m_ds(DriverStation::GetInstance()) {
GenericHID::GenericHID(int port) : m_ds(DirectJoystick_Interface::GetInstance()) {
	//if (port >= DriverStation::kJoystickPorts) {
	//	wpi_setWPIError(BadJoystickIndex);
	//}
	m_port = port;
}

bool GenericHID::GetRawButton(int button) const {
	return m_ds.GetStickButton(m_port, button);
}

bool GenericHID::GetRawButtonPressed(int button) {
	return m_ds.GetStickButtonPressed(m_port, button);
}

bool GenericHID::GetRawButtonReleased(int button) {
	return m_ds.GetStickButtonReleased(m_port, button);
}

double GenericHID::GetRawAxis(int axis) const {
	return m_ds.GetStickAxis(m_port, axis);
}

int GenericHID::GetPOV(int pov) const { return m_ds.GetStickPOV(m_port, pov); }

int GenericHID::GetAxisCount() const { return m_ds.GetStickAxisCount(m_port); }

int GenericHID::GetPOVCount() const { return m_ds.GetStickPOVCount(m_port); }

int GenericHID::GetButtonCount() const {
	return m_ds.GetStickButtonCount(m_port);
}

GenericHID::HIDType GenericHID::GetType() const {
	return static_cast<HIDType>(m_ds.GetJoystickType(m_port));
}

std::string GenericHID::GetName() const { return m_ds.GetJoystickName(m_port); }

int GenericHID::GetAxisType(int axis) const {
	return m_ds.GetJoystickAxisType(m_port, axis);
}

int GenericHID::GetPort() const { return m_port; }

void GenericHID::SetOutput(int outputNumber, bool value) {
	m_outputs =
		(m_outputs & ~(1 << (outputNumber - 1))) | (value << (outputNumber - 1));

	HAL_SetJoystickOutputs(m_port, m_outputs, m_leftRumble, m_rightRumble);
}

void GenericHID::SetOutputs(int value) {
	m_outputs = value;
	HAL_SetJoystickOutputs(m_port, m_outputs, m_leftRumble, m_rightRumble);
}

void GenericHID::SetRumble(RumbleType type, double value) {
	if (value < 0)
		value = 0;
	else if (value > 1)
		value = 1;
	if (type == kLeftRumble) {
		m_leftRumble = (uint16_t)(value * 65535.0);
	}
	else {
		m_rightRumble = uint16_t(value * 65535.0);
	}
	HAL_SetJoystickOutputs(m_port, m_outputs, m_leftRumble, m_rightRumble);
}


  /*******************************************************************************************************************************/
 /*															Joystick															*/
/*******************************************************************************************************************************/


Joystick::Joystick(int port) : GenericHID(port) {
	m_axes[Axis::kX] = kDefaultXChannel;
	m_axes[Axis::kY] = kDefaultYChannel;
	m_axes[Axis::kZ] = kDefaultZChannel;
	m_axes[Axis::kTwist] = kDefaultTwistChannel;
	m_axes[Axis::kThrottle] = kDefaultThrottleChannel;

	//HAL_Report(HALUsageReporting::kResourceType_Joystick, port);
}

void Joystick::SetXChannel(int channel) { m_axes[Axis::kX] = channel; }

void Joystick::SetYChannel(int channel) { m_axes[Axis::kY] = channel; }

void Joystick::SetZChannel(int channel) { m_axes[Axis::kZ] = channel; }

void Joystick::SetTwistChannel(int channel) { m_axes[Axis::kTwist] = channel; }

void Joystick::SetThrottleChannel(int channel) {
	m_axes[Axis::kThrottle] = channel;
}

void Joystick::SetAxisChannel(AxisType axis, int channel) {
	m_axes[axis] = channel;
}

int Joystick::GetXChannel() const { return m_axes[Axis::kX]; }

int Joystick::GetYChannel() const { return m_axes[Axis::kY]; }

int Joystick::GetZChannel() const { return m_axes[Axis::kZ]; }

int Joystick::GetTwistChannel() const { return m_axes[Axis::kTwist]; }

int Joystick::GetThrottleChannel() const { return m_axes[Axis::kThrottle]; }

double Joystick::GetX(JoystickHand hand) const {
	return GetRawAxis(m_axes[Axis::kX]);
}

double Joystick::GetY(JoystickHand hand) const {
	return GetRawAxis(m_axes[Axis::kY]);
}

double Joystick::GetZ() const { return GetRawAxis(m_axes[Axis::kZ]); }

double Joystick::GetTwist() const { return GetRawAxis(m_axes[Axis::kTwist]); }

double Joystick::GetThrottle() const {
	return GetRawAxis(m_axes[Axis::kThrottle]);
}

double Joystick::GetAxis(AxisType axis) const {
	switch (axis) {
	case kXAxis:
		return GetX();
	case kYAxis:
		return GetY();
	case kZAxis:
		return GetZ();
	case kTwistAxis:
		return GetTwist();
	case kThrottleAxis:
		return GetThrottle();
	default:
		//wpi_setWPIError(BadJoystickAxis);
		return 0.0;
	}
}

bool Joystick::GetTrigger() const { return GetRawButton(Button::kTrigger); }

bool Joystick::GetTriggerPressed() {
	return GetRawButtonPressed(Button::kTrigger);
}

bool Joystick::GetTriggerReleased() {
	return GetRawButtonReleased(Button::kTrigger);
}

bool Joystick::GetTop() const { return GetRawButton(Button::kTop); }

bool Joystick::GetTopPressed() { return GetRawButtonPressed(Button::kTop); }

bool Joystick::GetTopReleased() { return GetRawButtonReleased(Button::kTop); }

Joystick* Joystick::GetStickForPort(int port) {
	static std::array<std::unique_ptr<Joystick>, DriverStation::kJoystickPorts>
		joysticks{};
	auto stick = joysticks[port].get();
	if (stick == nullptr) {
		joysticks[port] = std::make_unique<Joystick>(port);
		stick = joysticks[port].get();
	}
	return stick;
}

bool Joystick::GetButton(ButtonType button) const {
	int temp = button;
	return GetRawButton(static_cast<Button>(temp));
}

double Joystick::GetMagnitude() const {
	return std::sqrt(std::pow(GetX(), 2) + std::pow(GetY(), 2));
}

double Joystick::GetDirectionRadians() const {
	return std::atan2(GetX(), -GetY());
}

double Joystick::GetDirectionDegrees() const {
	return (180 / kPi) * GetDirectionRadians();
}
