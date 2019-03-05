#pragma once

#include <array>

//#include <wpi/deprecated.h>
#ifndef WPI_DEPRECATED
#define WPI_DEPRECATED(msg)
#endif

//#include "frc/GenericHID.h"

namespace frc {

	class DirectJoystick_Interface {
	private:
		enum GameMode
		{
			eAuton,
			eTeleop,
			eTest
		};
		static bool DefaultIsEnabledCallback(void)
		{
			return false;
		}
		static int DefaultGameModeCallback(void)
		{
			return DirectJoystick_Interface::eTeleop;
		}

		//using std::function<bool()> as SimpleCallbackProto;
		std::function<bool(void)> m_IsEnabledCallback = DefaultIsEnabledCallback;
		std::function<int(void)> m_GameModeCallback = DefaultGameModeCallback;
	public:
		bool IsEnabled() const { return m_IsEnabledCallback(); }
		bool IsDisabled() const { return !IsEnabled(); }
		bool IsAutonomous() const { return m_GameModeCallback() == eAuton; }
		bool IsOperatorControl() const { return m_GameModeCallback() == eTeleop; }
		bool IsTest() const { return m_GameModeCallback() == eTest; }

	public:
		//Client code set these up
		void SetIsEnabledCallback(std::function<bool(void)> callback) { m_IsEnabledCallback = callback; }
		void SetIsGameModeCallback(std::function<int(void)> callback) { m_GameModeCallback = callback; }


		DirectJoystick_Interface() {}
		~DirectJoystick_Interface() {}

		DirectJoystick_Interface(const DirectJoystick_Interface&) = delete;
		DirectJoystick_Interface& operator=(const DirectJoystick_Interface&) = delete;

		static DirectJoystick_Interface& GetInstance();
		//static void ReportError(const wpi::Twine& error);
		//static void ReportWarning(const wpi::Twine& error);
		//static void ReportError(bool isError, int code, const wpi::Twine& error,const wpi::Twine& location, const wpi::Twine& stack);
		static constexpr int kJoystickPorts = 6;
		bool GetStickButton(int stick, int button);
		bool GetStickButtonPressed(int stick, int button);
		bool GetStickButtonReleased(int stick, int button);
		double GetStickAxis(int stick, int axis);
		int GetStickPOV(int stick, int pov);
		int GetStickButtons(int stick) const;
		int GetStickAxisCount(int stick) const;
		int GetStickPOVCount(int stick) const;
		int GetStickButtonCount(int stick) const;
		bool GetJoystickIsXbox(int stick) const;
		int GetJoystickType(int stick) const;
		std::string GetJoystickName(int stick) const;
		int GetJoystickAxisType(int stick, int axis) const;
		bool IsDSAttached() const { return false; }
	};

	using DriverStation = DirectJoystick_Interface;
	//typedef DirectJoystick_Interface DriverStation;

	/**
	 * GenericHID Interface.
	 */
	//class GenericHID : public ErrorBase {
	class GenericHID  {
	public:
		enum RumbleType { kLeftRumble, kRightRumble };

		enum HIDType {
			kUnknown = -1,
			kXInputUnknown = 0,
			kXInputGamepad = 1,
			kXInputWheel = 2,
			kXInputArcadeStick = 3,
			kXInputFlightStick = 4,
			kXInputDancePad = 5,
			kXInputGuitar = 6,
			kXInputGuitar2 = 7,
			kXInputDrumKit = 8,
			kXInputGuitar3 = 11,
			kXInputArcadePad = 19,
			kHIDJoystick = 20,
			kHIDGamepad = 21,
			kHIDDriving = 22,
			kHIDFlight = 23,
			kHID1stPerson = 24
		};

		enum JoystickHand { kLeftHand = 0, kRightHand = 1 };

		explicit GenericHID(int port);
		virtual ~GenericHID() = default;

		GenericHID(GenericHID&&) = default;
		GenericHID& operator=(GenericHID&&) = default;

		virtual double GetX(JoystickHand hand = kRightHand) const = 0;
		virtual double GetY(JoystickHand hand = kRightHand) const = 0;

		/**
		 * Get the button value (starting at button 1).
		 *
		 * The buttons are returned in a single 16 bit value with one bit representing
		 * the state of each button. The appropriate button is returned as a boolean
		 * value.
		 *
		 * @param button The button number to be read (starting at 1)
		 * @return The state of the button.
		 */
		bool GetRawButton(int button) const;

		/**
		 * Whether the button was pressed since the last check. Button indexes begin
		 * at 1.
		 *
		 * @param button The button index, beginning at 1.
		 * @return Whether the button was pressed since the last check.
		 */
		bool GetRawButtonPressed(int button);

		/**
		 * Whether the button was released since the last check. Button indexes begin
		 * at 1.
		 *
		 * @param button The button index, beginning at 1.
		 * @return Whether the button was released since the last check.
		 */
		bool GetRawButtonReleased(int button);

		/**
		 * Get the value of the axis.
		 *
		 * @param axis The axis to read, starting at 0.
		 * @return The value of the axis.
		 */
		double GetRawAxis(int axis) const;

		/**
		 * Get the angle in degrees of a POV on the HID.
		 *
		 * The POV angles start at 0 in the up direction, and increase clockwise
		 * (e.g. right is 90, upper-left is 315).
		 *
		 * @param pov The index of the POV to read (starting at 0)
		 * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
		 */
		int GetPOV(int pov = 0) const;

		/**
		 * Get the number of axes for the HID.
		 *
		 * @return the number of axis for the current HID
		 */
		int GetAxisCount() const;

		/**
		 * Get the number of POVs for the HID.
		 *
		 * @return the number of POVs for the current HID
		 */
		int GetPOVCount() const;

		/**
		 * Get the number of buttons for the HID.
		 *
		 * @return the number of buttons on the current HID
		 */
		int GetButtonCount() const;

		/**
		 * Get the type of the HID.
		 *
		 * @return the type of the HID.
		 */
		GenericHID::HIDType GetType() const;

		/**
		 * Get the name of the HID.
		 *
		 * @return the name of the HID.
		 */
		std::string GetName() const;

		/**
		 * Get the axis type of a joystick axis.
		 *
		 * @return the axis type of a joystick axis.
		 */
		int GetAxisType(int axis) const;

		/**
		 * Get the port number of the HID.
		 *
		 * @return The port number of the HID.
		 */
		int GetPort() const;

		/**
		 * Set a single HID output value for the HID.
		 *
		 * @param outputNumber The index of the output to set (1-32)
		 * @param value        The value to set the output to
		 */
		void SetOutput(int outputNumber, bool value);

		/**
		 * Set all output values for the HID.
		 *
		 * @param value The 32 bit output value (1 bit for each output)
		 */
		void SetOutputs(int value);

		/**
		 * Set the rumble output for the HID.
		 *
		 * The DS currently supports 2 rumble values, left rumble and right rumble.
		 *
		 * @param type  Which rumble value to set
		 * @param value The normalized value (0 to 1) to set the rumble to
		 */
		void SetRumble(RumbleType type, double value);

	private:
		DirectJoystick_Interface& m_ds;
		int m_port;
		int m_outputs = 0;
		uint16_t m_leftRumble = 0;
		uint16_t m_rightRumble = 0;
	};




	/**
	 * Handle input from standard Joysticks connected to the Driver Station.
	 *
	 * This class handles standard input that comes from the Driver Station. Each
	 * time a value is requested the most recent value is returned. There is a
	 * single class instance for each joystick and the mapping of ports to hardware
	 * buttons depends on the code in the Driver Station.
	 */
	class Joystick : public GenericHID {
	public:
		static constexpr int kDefaultXChannel = 0;
		static constexpr int kDefaultYChannel = 1;
		static constexpr int kDefaultZChannel = 2;
		static constexpr int kDefaultTwistChannel = 2;
		static constexpr int kDefaultThrottleChannel = 3;

		WPI_DEPRECATED("Use kDefaultXChannel instead.")
			static constexpr int kDefaultXAxis = 0;
		WPI_DEPRECATED("Use kDefaultYChannel instead.")
			static constexpr int kDefaultYAxis = 1;
		WPI_DEPRECATED("Use kDefaultZChannel instead.")
			static constexpr int kDefaultZAxis = 2;
		WPI_DEPRECATED("Use kDefaultTwistChannel instead.")
			static constexpr int kDefaultTwistAxis = 2;
		WPI_DEPRECATED("Use kDefaultThrottleChannel instead.")
			static constexpr int kDefaultThrottleAxis = 3;

		enum AxisType { kXAxis, kYAxis, kZAxis, kTwistAxis, kThrottleAxis };
		enum ButtonType { kTriggerButton, kTopButton };

		/**
		 * Construct an instance of a joystick.
		 *
		 * The joystick index is the USB port on the Driver Station.
		 *
		 * @param port The port on the Driver Station that the joystick is plugged
		 *             into (0-5).
		 */
		explicit Joystick(int port);

		virtual ~Joystick() = default;

		Joystick(Joystick&&) = default;
		Joystick& operator=(Joystick&&) = default;

		/**
		 * Set the channel associated with the X axis.
		 *
		 * @param channel The channel to set the axis to.
		 */
		void SetXChannel(int channel);

		/**
		 * Set the channel associated with the Y axis.
		 *
		 * @param axis    The axis to set the channel for.
		 * @param channel The channel to set the axis to.
		 */
		void SetYChannel(int channel);

		/**
		 * Set the channel associated with the Z axis.
		 *
		 * @param axis    The axis to set the channel for.
		 * @param channel The channel to set the axis to.
		 */
		void SetZChannel(int channel);

		/**
		 * Set the channel associated with the twist axis.
		 *
		 * @param axis    The axis to set the channel for.
		 * @param channel The channel to set the axis to.
		 */
		void SetTwistChannel(int channel);

		/**
		 * Set the channel associated with the throttle axis.
		 *
		 * @param axis    The axis to set the channel for.
		 * @param channel The channel to set the axis to.
		 */
		void SetThrottleChannel(int channel);

		/**
		 * Set the channel associated with a specified axis.
		 *
		 * @param axis    The axis to set the channel for.
		 * @param channel The channel to set the axis to.
		 */
		WPI_DEPRECATED("Use the more specific axis channel setter functions.")
			void SetAxisChannel(AxisType axis, int channel);

		/**
		 * Get the channel currently associated with the X axis.
		 *
		 * @return The channel for the axis.
		 */
		int GetXChannel() const;

		/**
		 * Get the channel currently associated with the Y axis.
		 *
		 * @return The channel for the axis.
		 */
		int GetYChannel() const;

		/**
		 * Get the channel currently associated with the Z axis.
		 *
		 * @return The channel for the axis.
		 */
		int GetZChannel() const;

		/**
		 * Get the channel currently associated with the twist axis.
		 *
		 * @return The channel for the axis.
		 */
		int GetTwistChannel() const;

		/**
		 * Get the channel currently associated with the throttle axis.
		 *
		 * @return The channel for the axis.
		 */
		int GetThrottleChannel() const;

		/**
		 * Get the X value of the joystick.
		 *
		 * This depends on the mapping of the joystick connected to the current port.
		 *
		 * @param hand This parameter is ignored for the Joystick class and is only
		 *             here to complete the GenericHID interface.
		 */
		double GetX(JoystickHand hand = kRightHand) const override;

		/**
		 * Get the Y value of the joystick.
		 *
		 * This depends on the mapping of the joystick connected to the current port.
		 *
		 * @param hand This parameter is ignored for the Joystick class and is only
		 *             here to complete the GenericHID interface.
		 */
		double GetY(JoystickHand hand = kRightHand) const override;

		/**
		 * Get the Z value of the current joystick.
		 *
		 * This depends on the mapping of the joystick connected to the current port.
		 */
		double GetZ() const;

		/**
		 * Get the twist value of the current joystick.
		 *
		 * This depends on the mapping of the joystick connected to the current port.
		 */
		double GetTwist() const;

		/**
		 * Get the throttle value of the current joystick.
		 *
		 * This depends on the mapping of the joystick connected to the current port.
		 */
		double GetThrottle() const;

		/**
		 * For the current joystick, return the axis determined by the argument.
		 *
		 * This is for cases where the joystick axis is returned programatically,
		 * otherwise one of the previous functions would be preferable (for example
		 * GetX()).
		 *
		 * @param axis The axis to read.
		 * @return The value of the axis.
		 */
		WPI_DEPRECATED("Use the more specific axis channel getter functions.")
			double GetAxis(AxisType axis) const;

		/**
		 * Read the state of the trigger on the joystick.
		 *
		 * Look up which button has been assigned to the trigger and read its state.
		 *
		 * @return The state of the trigger.
		 */
		bool GetTrigger() const;

		/**
		 * Whether the trigger was pressed since the last check.
		 *
		 * @return Whether the button was pressed since the last check.
		 */
		bool GetTriggerPressed();

		/**
		 * Whether the trigger was released since the last check.
		 *
		 * @return Whether the button was released since the last check.
		 */
		bool GetTriggerReleased();

		/**
		 * Read the state of the top button on the joystick.
		 *
		 * Look up which button has been assigned to the top and read its state.
		 *
		 * @return The state of the top button.
		 */
		bool GetTop() const;

		/**
		 * Whether the top button was pressed since the last check.
		 *
		 * @return Whether the button was pressed since the last check.
		 */
		bool GetTopPressed();

		/**
		 * Whether the top button was released since the last check.
		 *
		 * @return Whether the button was released since the last check.
		 */
		bool GetTopReleased();

		WPI_DEPRECATED("Use Joystick instances instead.")
			static Joystick* GetStickForPort(int port);

		/**
		 * Get buttons based on an enumerated type.
		 *
		 * The button type will be looked up in the list of buttons and then read.
		 *
		 * @param button The type of button to read.
		 * @return The state of the button.
		 */
		WPI_DEPRECATED("Use the more specific button getter functions.")
			bool GetButton(ButtonType button) const;

		/**
		 * Get the magnitude of the direction vector formed by the joystick's
		 * current position relative to its origin.
		 *
		 * @return The magnitude of the direction vector
		 */
		double GetMagnitude() const;

		/**
		 * Get the direction of the vector formed by the joystick and its origin
		 * in radians.
		 *
		 * @return The direction of the vector in radians
		 */
		double GetDirectionRadians() const;

		/**
		 * Get the direction of the vector formed by the joystick and its origin
		 * in degrees.
		 *
		 * @return The direction of the vector in degrees
		 */
		double GetDirectionDegrees() const;

	private:
		enum Axis { kX, kY, kZ, kTwist, kThrottle, kNumAxes };
		enum Button { kTrigger = 1, kTop = 2 };

		std::array<int, Axis::kNumAxes> m_axes;
	};

}  // namespace frc
