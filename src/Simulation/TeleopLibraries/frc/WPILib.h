#pragma once
#include <thread>
#include <chrono>

//as a start we'll include the same controls used in the robot libraries
#define COMMON_API
#include "../../RobotLibraries/Base/Base_Includes.h"
#include "../../RobotLibraries/Base/Script.h"
//#include "../../../main/cpp/Base/Time_Type.h"

#include "../../RobotLibraries/Common/Robot_Control_Common.h"
#include <frc/Joystick.h>
#include "../..//RobotLibraries/Common/SmartDashboard.h"

namespace frc
{
	#if 0
	class VictorSP : public PWMSpeedController
	{
	public:
		/**
		 * Constructor for a VictorSP.
		 *
		 * @param channel The PWM channel that the VictorSP is attached to. 0-9 are
		 *                on-board, 10-19 are on the MXP port
		 */
		VictorSP(int channel) : PWMSpeedController(0, channel, "VictorSP")  //Note: the name will need to be merged after construction
		{
		}

		VictorSP(VictorSP&&) = default;
		VictorSP& operator=(VictorSP&&) = default;
	};
	#endif


	class SensorCollection {
	public:
		int GetAnalogIn() { return 0; }
		ErrorCode SetAnalogPosition(int newPosition, int timeoutMs = 0) { return OK; }
		int GetAnalogInRaw() { return 0; }
		int GetAnalogInVel() { return 0; }
		int GetQuadraturePosition() { return 0; }
		ErrorCode SetQuadraturePosition(int newPosition, int timeoutMs = 0) { return OK; }
		ErrorCode SyncQuadratureWithPulseWidth(int bookend0, int bookend1, bool bCrossZeroOnInterval, int offset = 0, int timeoutMs = 0) { return OK; }
		int GetQuadratureVelocity() { return 0; }
		int GetPulseWidthPosition() { return 0; }
		ErrorCode SetPulseWidthPosition(int newPosition, int timeoutMs = 0) { return OK; }
		int GetPulseWidthVelocity() { return 0; }
		int GetPulseWidthRiseToFallUs() { return 0; }
		int GetPulseWidthRiseToRiseUs() { return 0; }
		int GetPinStateQuadA() { return 0; }
		int GetPinStateQuadB() { return 0; }
		int GetPinStateQuadIdx() { return 0; }
		int IsFwdLimitSwitchClosed() { return 0; }
		int IsRevLimitSwitchClosed() { return 0; }
	};

	class TalonSRX : public VictorSPX
	{
	private:
		SensorCollection m_SensorCollectionPacify;
		public:
			TalonSRX(int channel) : VictorSPX(channel, "TalonSRX")  //Note: the name will need to be merged after construction
			{
			}

			SensorCollection & GetSensorCollection() {		return m_SensorCollectionPacify; }
	};


	class AnalogPotentiometer : public AnalogChannel
	{
	public:
		AnalogPotentiometer(int channel) : AnalogChannel(0,channel,"AnalogPotentiometer")
		{}
		double Get() { return (double)GetAverageValue(); }
	};

	class Encoder : public Encoder2
	{
	public:
		Encoder(int AChannel,int BChannel,bool Reversed) : Encoder2(0,AChannel,BChannel,"Encoder")
		{}
	};

	//class PowerDistributionPanel : public ErrorBase, public SendableBase {
	class PowerDistributionPanel {
	public:
		PowerDistributionPanel() {}
		explicit PowerDistributionPanel(int module) {}

		PowerDistributionPanel(PowerDistributionPanel&&) = default;
		PowerDistributionPanel& operator=(PowerDistributionPanel&&) = default;
		double GetVoltage() const { return 0.0; }
		double GetTemperature() const { return 0.0; }
		double GetCurrent(int channel) const { return 0.0; }
		double GetTotalCurrent() const { return 0.0; }
		double GetTotalPower() const { return 0.0; }
		double GetTotalEnergy() const { return 0.0; }
		void ResetTotalEnergy() {}
		void ClearStickyFaults() {}
		//void InitSendable(SendableBuilder& builder) override;
		//void InitSendable(SendableBuilder& builder) {}
	private:
		//HAL_PDPHandle m_handle = HAL_kInvalidHandle;
	};

}

//TODO implement
namespace frc {
	__inline void Wait(double seconds)
	{
		std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
	}

	__inline double GetTime() {
		using std::chrono::duration;
		using std::chrono::duration_cast;
		using std::chrono::system_clock;

		return duration_cast<duration<double>>(system_clock::now().time_since_epoch())
			.count();
	}

	int32_t HAL_SetJoystickOutputs(int32_t joystickNum, int64_t outputs, int32_t leftRumble, int32_t rightRumble);
}

namespace cs
{
	class UsbCamera
	{
	public:
		void SetExposureAuto() {}
		bool SetResolution(int width, int height) {	return false; }
		bool SetFPS(int fps) { return false; }
	};
	class AxisCamera
	{

	};
}


namespace frc {

	/**
	 * Singleton class for creating and keeping camera servers.
	 *
	 * Also publishes camera information to NetworkTables.
	 */
	class CameraServer {
	public:
		static constexpr uint16_t kBasePort = 1181;
		static constexpr int kSize640x480 = 0;
		static constexpr int kSize320x240 = 1;
		static constexpr int kSize160x120 = 2;

		static CameraServer* GetInstance()
		{
			static CameraServer cs;
			return &cs;
		}
		cs::UsbCamera StartAutomaticCapture() 
		{
			static cs::UsbCamera instance;
			return instance;
		}
		cs::UsbCamera StartAutomaticCapture(int dev)
		{
			static cs::UsbCamera instance;
			return instance;
		}

		//cs::UsbCamera StartAutomaticCapture(const wpi::Twine& name, int dev);
		//cs::UsbCamera StartAutomaticCapture(const wpi::Twine& name,	const wpi::Twine& path);
		//void StartAutomaticCapture(const cs::VideoSource& camera);
		//cs::AxisCamera AddAxisCamera(const wpi::Twine& host);
		cs::AxisCamera AddAxisCamera(const char* host)
		{
			static cs::AxisCamera instance;
			return instance;
		}

		cs::AxisCamera AddAxisCamera(const std::string& host)
		{
			static cs::AxisCamera instance;
			return instance;
		}

		//cs::AxisCamera AddAxisCamera(wpi::ArrayRef<std::string> hosts);
		//template <typename T>
		//cs::AxisCamera AddAxisCamera(std::initializer_list<T> hosts);
		//cs::AxisCamera AddAxisCamera(const wpi::Twine& name, const wpi::Twine& host);
		//cs::AxisCamera AddAxisCamera(const wpi::Twine& name, const char* host);
		//cs::AxisCamera AddAxisCamera(const wpi::Twine& name, const std::string& host);
		//cs::AxisCamera AddAxisCamera(const wpi::Twine& name,
		//	wpi::ArrayRef<std::string> hosts);
		//template <typename T>
		//cs::AxisCamera AddAxisCamera(const wpi::Twine& name,
		//	std::initializer_list<T> hosts);
		//cs::CvSink GetVideo();
		//cs::CvSink GetVideo(const cs::VideoSource& camera);
		//cs::CvSink GetVideo(const wpi::Twine& name);
		//cs::CvSource PutVideo(const wpi::Twine& name, int width, int height);
		//cs::MjpegServer AddServer(const wpi::Twine& name);
		//cs::MjpegServer AddServer(const wpi::Twine& name, int port);
		//void AddServer(const cs::VideoSink& server);
		//void RemoveServer(const wpi::Twine& name);
		//cs::VideoSink GetServer();
		//cs::VideoSink GetServer(const wpi::Twine& name);
		//void AddCamera(const cs::VideoSource& camera);
		void RemoveCamera(std::string name) {}
		void SetSize(int size);

	private:
		CameraServer() {}
		~CameraServer() {}

		//struct Impl;
		//std::unique_ptr<Impl> m_impl;
	};

}  // namespace frc
