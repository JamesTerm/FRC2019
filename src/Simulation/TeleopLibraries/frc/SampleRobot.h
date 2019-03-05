#pragma once
#include <future>
#include <thread>
#include <chrono>

#define START_ROBOT_CLASS(x)

namespace frc {

	class RobotBase {
	private:
		enum GameMode
		{
			eAuton,
			eTeleop,
			eTest
		};
		static bool DefaultIsEnabledCallback(void)
		{	return false;
		}
		static int DefaultGameModeCallback(void)
		{	return RobotBase::eTest;
		}

		//using std::function<bool()> as SimpleCallbackProto;
		std::function<bool(void)> m_IsEnabledCallback= DefaultIsEnabledCallback;
		std::function<int(void)> m_GameModeCallback= DefaultGameModeCallback;
	public:
		//Client code set these up
		void SetIsEnabledCallback(std::function<bool (void)> callback) 
		{ 
			m_IsEnabledCallback = callback; 
			DriverStation::GetInstance().SetIsEnabledCallback(callback);   // pass it to DS
		}
		void SetIsGameModeCallback(std::function<int (void)> callback) 
		{ 
			m_GameModeCallback = callback; 
			DriverStation::GetInstance().SetIsGameModeCallback(callback);   // pass it to DS
		}
		/**
		 * Determine if the Robot is currently enabled.
		 *
		 * @return True if the Robot is currently enabled by the field controls.
		 */
		bool IsEnabled() const { return m_IsEnabledCallback(); }

		/**
		 * Determine if the Robot is currently disabled.
		 *
		 * @return True if the Robot is currently disabled by the field controls.
		 */
		bool IsDisabled() const { return !IsEnabled(); }

		/**
		 * Determine if the robot is currently in Autonomous mode.
		 *
		 * @return True if the robot is currently operating Autonomously as determined
		 *         by the field controls.
		 */
		bool IsAutonomous() const { return m_GameModeCallback() == eAuton; }

		/**
		 * Determine if the robot is currently in Operator Control mode.
		 *
		 * @return True if the robot is currently operating in Tele-Op mode as
		 *         determined by the field controls.
		 */
		bool IsOperatorControl() const { return m_GameModeCallback() == eTeleop; }

		/**
		 * Determine if the robot is currently in Test mode.
		 *
		 * @return True if the robot is currently running tests as determined by the
		 *         field controls.
		 */
		bool IsTest() const { return m_GameModeCallback() == eTest; }

		/**
		 * Indicates if new data is available from the driver station.
		 *
		 * @return Has new data arrived over the network since the last time this
		 *         function was called?
		 */
		bool IsNewDataAvailable() const { return true; }

		/**
		 * Gets the ID of the main robot thread.
		 */
		//static std::thread::id GetThreadId();

		virtual void StartCompetition() = 0;

		static constexpr bool IsReal() { return false; }
		static constexpr bool IsSimulation() { return !IsReal(); }

	protected:
		RobotBase() {}
		virtual ~RobotBase() {}

		//RobotBase(RobotBase&&);
		//RobotBase& operator=(RobotBase&&);

		//DriverStation& m_ds;

		//static std::thread::id m_threadId;
	};

	class SampleRobot : public RobotBase {
	public:
		/**
		 * Start a competition.
		 *
		 * This code needs to track the order of the field starting to ensure that
		 * everything happens in the right order. Repeatedly run the correct method,
		 * either Autonomous or OperatorControl or Test when the robot is enabled.
		 * After running the correct method, wait for some state to change, either the
		 * other mode starts or the robot is disabled. Then go back and wait for the
		 * robot to be enabled again.
		 */
		//void StartCompetition() override;
		void StartCompetition() {}

		/**
		 * Robot-wide initialization code should go here.
		 *
		 * Users should override this method for default Robot-wide initialization
		 * which will be called when the robot is first powered on. It will be called
		 * exactly one time.
		 *
		 * Warning: the Driver Station "Robot Code" light and FMS "Robot Ready"
		 * indicators will be off until RobotInit() exits. Code in RobotInit() that
		 * waits for enable will cause the robot to never indicate that the code is
		 * ready, causing the robot to be bypassed in a match.
		 */
		virtual void RobotInit() {}

		/**
		 * Disabled should go here.
		 *
		 * Programmers should override this method to run code that should run while
		 * the field is disabled.
		 */
		virtual void Disabled() {}

		/**
		 * Autonomous should go here.
		 *
		 * Programmers should override this method to run code that should run while
		 * the field is in the autonomous period. This will be called once each time
		 * the robot enters the autonomous state.
		 */
		virtual void Autonomous() {}

		/**
		 * Operator control (tele-operated) code should go here.
		 *
		 * Programmers should override this method to run code that should run while
		 * the field is in the Operator Control (tele-operated) period. This is called
		 * once each time the robot enters the teleop state.
		 */
		virtual void OperatorControl() {}

		/**
		 * Test program should go here.
		 *
		 * Programmers should override this method to run code that executes while the
		 * robot is in test mode. This will be called once whenever the robot enters
		 * test mode
		 */
		virtual void Test() {}

		/**
		 * Robot main program for free-form programs.
		 *
		 * This should be overridden by user subclasses if the intent is to not use
		 * the Autonomous() and OperatorControl() methods. In that case, the program
		 * is responsible for sensing when to run the autonomous and operator control
		 * functions in their program.
		 *
		 * This method will be called immediately after the constructor is called. If
		 * it has not been overridden by a user subclass (i.e. the default version
		 * runs), then the Autonomous() and OperatorControl() methods will be called.
		 */
		virtual void RobotMain() {}

	protected:
		WPI_DEPRECATED(
			"WARNING: While it may look like a good choice to use for your code if "
			"you're inexperienced, don't. Unless you know what you are doing, "
			"complex code will be much more difficult under this system. Use "
			"TimedRobot or Command-Based instead.")
			SampleRobot() {}
		virtual ~SampleRobot() = default;

		SampleRobot(SampleRobot&&) = default;
		SampleRobot& operator=(SampleRobot&&) = default;

	//private:
	//	bool m_robotMainOverridden = true;
	};

}  // namespace frc
