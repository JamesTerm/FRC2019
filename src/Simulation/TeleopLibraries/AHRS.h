#pragma once
#include <thread>

class AHRSInternal
{
	AHRSInternal()
	{}
};

using InertialDataIntegrator = int;
using ContinuousAngleTracker = int;
using OffsetTracker = int;
using IIOProvider = int;
using ITimestampedDataSubscriber = int;


//class SPI : public ErrorBase {
class SPI {
public:
	enum Port { kOnboardCS0 = 0, kOnboardCS1, kOnboardCS2, kOnboardCS3, kMXP };

	explicit SPI(Port port) {}
	//~SPI() override {}
	~SPI()  {}

	SPI(const SPI&) = delete;
	SPI& operator=(const SPI&) = delete;

	void SetClockRate(double hz) {}

	void SetMSBFirst() {}
	void SetLSBFirst() {}

	void SetSampleDataOnFalling() {}
	void SetSampleDataOnRising() {}

	void SetClockActiveLow() {}
	void SetClockActiveHigh() {}

	void SetChipSelectActiveHigh() {}
	void SetChipSelectActiveLow() {}

	virtual int Write(uint8_t* data, int size) { return 0; }
	virtual int Read(bool initiate, uint8_t* dataReceived, int size) { return 0; }
	virtual int Transaction(uint8_t* dataToSend, uint8_t* dataReceived, int size) { return 0; }

	void InitAuto(int bufferSize) {}
	void FreeAuto() {}
	//void SetAutoTransmitData(llvm::ArrayRef<uint8_t> dataToSend, int zeroSize) {}
	void StartAutoRate(double period) {}
	//void StartAutoTrigger(DigitalSource& source, bool rising, bool falling){}
	void StopAuto(){}
	void ForceAutoRead(){}
	int ReadAutoReceivedData(uint8_t* buffer, int numToRead, double timeout) { return 0; }
	int GetAutoDroppedCount() { return 0; }

	void InitAccumulator(double period, int cmd, int xferSize, int validMask,
		int validValue, int dataShift, int dataSize,
		bool isSigned, bool bigEndian) {}
	void FreeAccumulator() {}
	void ResetAccumulator() {}
	void SetAccumulatorCenter(int center) {}
	void SetAccumulatorDeadband(int deadband) {}
	int GetAccumulatorLastValue() const { return 0; }
	int64_t GetAccumulatorValue() const { return 0; }
	int64_t GetAccumulatorCount() const { return 0; }
	double GetAccumulatorAverage() const { return 0.0; }
	void GetAccumulatorOutput(int64_t& value, int64_t& count) const {}

//protected:
//	HAL_SPIPort m_port;
//	bool m_msbFirst = false;          // Default little-endian
//	bool m_sampleOnTrailing = false;  // Default data updated on falling edge
//	bool m_clk_idle_high = false;     // Default clock active high
//
//private:
//	void Init();
//
//	class Accumulator;
//	std::unique_ptr<Accumulator> m_accum;
};


//class I2C : public ErrorBase {
class I2C 
{
public:
	enum Port { kOnboard = 0, kMXP };

	I2C(Port port, int deviceAddress) {}
	//~I2C() override {}
	~I2C()  {}

	I2C(const I2C&) = delete;
	I2C& operator=(const I2C&) = delete;

	bool Transaction(uint8_t* dataToSend, int sendSize, uint8_t* dataReceived, int receiveSize) { return false; }
	bool AddressOnly() { return false; }
	bool Write(int registerAddress, uint8_t data) { return false; }
	bool WriteBulk(uint8_t* data, int count) { return false; }
	bool Read(int registerAddress, int count, uint8_t* data) { return false; }
	bool ReadOnly(int count, uint8_t* buffer) { return false; }
	bool VerifySensor(int registerAddress, int count, const uint8_t* expected) { return false; }

//private:
//	HAL_I2CPort m_port;
//	int m_deviceAddress;
};

//class SerialPort : public ErrorBase {
class SerialPort
{
public:
	enum Parity {
		kParity_None = 0,
		kParity_Odd = 1,
		kParity_Even = 2,
		kParity_Mark = 3,
		kParity_Space = 4
	};
	enum StopBits {
		kStopBits_One = 10,
		kStopBits_OnePointFive = 15,
		kStopBits_Two = 20
	};
	enum FlowControl {
		kFlowControl_None = 0,
		kFlowControl_XonXoff = 1,
		kFlowControl_RtsCts = 2,
		kFlowControl_DtrDsr = 4
	};
	enum WriteBufferMode { kFlushOnAccess = 1, kFlushWhenFull = 2 };
	enum Port { kOnboard = 0, kMXP = 1, kUSB = 2, kUSB1 = 2, kUSB2 = 3 };

	SerialPort(int baudRate, Port port = kOnboard, int dataBits = 8,Parity parity = kParity_None, StopBits stopBits = kStopBits_One) {}
	//WPI_DEPRECATED("Will be removed for 2020")
	//SerialPort(int baudRate, const wpi::Twine& portName, Port port = kOnboard,int dataBits = 8, Parity parity = kParity_None,StopBits stopBits = kStopBits_One) {}
	~SerialPort() {}

	SerialPort(SerialPort&& rhs){}
	SerialPort& operator=(SerialPort&& rhs) { return *this; }
	void SetFlowControl(FlowControl flowControl) {}
	void EnableTermination(char terminator = '\n') {}
	void DisableTermination() {}
	int GetBytesReceived() {}
	int Read(char* buffer, int count) { return 0; }
	int Write(const char* buffer, int count) { return 0; }
	int Write(const char *buffer) { return 0; }
	void SetTimeout(double timeout){}
	void SetReadBufferSize(int size) {}
	void SetWriteBufferSize(int size) {}
	void SetWriteBufferMode(WriteBufferMode mode) {}
	void Flush() {}
	void Reset() {}
};


//class AHRS : public SendableBase,
//	public ErrorBase,
//	public PIDSource {
class AHRS 
{
public:

	enum BoardAxis {
		kBoardAxisX = 0,
		kBoardAxisY = 1,
		kBoardAxisZ = 2,
	};

	struct BoardYawAxis
	{
		/* Identifies one of the board axes */
		BoardAxis board_axis;
		/* true if axis is pointing up (with respect to gravity); false if pointing down. */
		bool up;
	};

	enum SerialDataType {
		/**
		 * (default):  6 and 9-axis processed data
		 */
		kProcessedData = 0,
		/**
		 * unprocessed data from each individual sensor
		 */
		kRawData = 1
	};

private:
	friend class AHRSInternal;
	AHRSInternal *      ahrs_internal;

	volatile float      yaw;
	volatile float      pitch;
	volatile float      roll;
	volatile float      compass_heading;
	volatile float      world_linear_accel_x;
	volatile float      world_linear_accel_y;
	volatile float      world_linear_accel_z;
	volatile float      mpu_temp_c;
	volatile float      fused_heading;
	volatile float      altitude;
	volatile float      baro_pressure;
	volatile bool       is_moving;
	volatile bool       is_rotating;
	volatile float      baro_sensor_temp_c;
	volatile bool       altitude_valid;
	volatile bool       is_magnetometer_calibrated;
	volatile bool       magnetic_disturbance;
	volatile float    	quaternionW;
	volatile float    	quaternionX;
	volatile float    	quaternionY;
	volatile float    	quaternionZ;

	/* Integrated Data */
	float velocity[3];
	float displacement[3];


	/* Raw Data */
	volatile int16_t    raw_gyro_x;
	volatile int16_t    raw_gyro_y;
	volatile int16_t    raw_gyro_z;
	volatile int16_t    raw_accel_x;
	volatile int16_t    raw_accel_y;
	volatile int16_t    raw_accel_z;
	volatile int16_t    cal_mag_x;
	volatile int16_t    cal_mag_y;
	volatile int16_t    cal_mag_z;

	/* Configuration/Status */
	volatile uint8_t    update_rate_hz;
	volatile int16_t    accel_fsr_g;
	volatile int16_t    gyro_fsr_dps;
	volatile int16_t    capability_flags;
	volatile uint8_t    op_status;
	volatile int16_t    sensor_status;
	volatile uint8_t    cal_status;
	volatile uint8_t    selftest_status;

	/* Board ID */
	volatile uint8_t    board_type;
	volatile uint8_t    hw_rev;
	volatile uint8_t    fw_ver_major;
	volatile uint8_t    fw_ver_minor;

	long                last_sensor_timestamp;
	double              last_update_time;

	InertialDataIntegrator *integrator;
	ContinuousAngleTracker *yaw_angle_tracker;
	OffsetTracker *         yaw_offset_tracker;
	IIOProvider *           io;

	std::thread *           task;

#define MAX_NUM_CALLBACKS 3
	ITimestampedDataSubscriber *callbacks[MAX_NUM_CALLBACKS];
	void *callback_contexts[MAX_NUM_CALLBACKS];

public:
	AHRS(SPI::Port spi_port_id) {}
	AHRS(I2C::Port i2c_port_id) {}
	AHRS(SerialPort::Port serial_port_id) {}

	AHRS(SPI::Port spi_port_id, uint8_t update_rate_hz) {}
	AHRS(SPI::Port spi_port_id, uint32_t spi_bitrate, uint8_t update_rate_hz) {}

	AHRS(I2C::Port i2c_port_id, uint8_t update_rate_hz) {}

	AHRS(SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz) {}

	float  GetPitch() { return 0.0; }
	float  GetRoll() { return 0.0; }
	float  GetYaw() { return 0.0; }
	float  GetCompassHeading() { return 0.0; }
	void   ZeroYaw() {}
	bool   IsCalibrating() { return false; }
	bool   IsConnected() { return false; }
	double GetByteCount() { return 0.0; }
	double GetUpdateCount() { return 0.0; }
	long   GetLastSensorTimestamp() { return 0; }
	float  GetWorldLinearAccelX() { return 0.0f; }
	float  GetWorldLinearAccelY() { return 0.0f; }
	float  GetWorldLinearAccelZ() { return 0.0f; }
	bool   IsMoving() { return false; }
	bool   IsRotating() { return false; }
	float  GetBarometricPressure() { return 0.0f; }
	float  GetAltitude() { return 0.0f; }
	bool   IsAltitudeValid() { return false; }
	float  GetFusedHeading() { return 0.0f; }
	bool   IsMagneticDisturbance() { return false; }
	bool   IsMagnetometerCalibrated() { return false; }
	float  GetQuaternionW() { return 0.0f; }
	float  GetQuaternionX() { return 0.0f; }
	float  GetQuaternionY() { return 0.0f; }
	float  GetQuaternionZ() { return 0.0f; }
	void   ResetDisplacement() {}
	void   UpdateDisplacement(float accel_x_g, float accel_y_g,	int update_rate_hz, bool is_moving) {}
	float  GetVelocityX() { return 0.0f; }
	float  GetVelocityY() { return 0.0f; }
	float  GetVelocityZ() { return 0.0f; }
	float  GetDisplacementX() { return 0.0f; }
	float  GetDisplacementY() { return 0.0f; }
	float  GetDisplacementZ() { return 0.0f; }
	double GetAngle() { return 0.0; }
	double GetRate() { return 0.0; }
	void   SetAngleAdjustment(double angle) {}
	double GetAngleAdjustment() { return 0.0; }
	void   Reset() {}
	float  GetRawGyroX() { return 0.0f; }
	float  GetRawGyroY() { return 0.0f; }
	float  GetRawGyroZ() { return 0.0f; }
	float  GetRawAccelX() { return 0.0f; }
	float  GetRawAccelY() { return 0.0f; }
	float  GetRawAccelZ() { return 0.0f; }
	float  GetRawMagX() { return 0.0f; }
	float  GetRawMagY() { return 0.0f; }
	float  GetRawMagZ() { return 0.0f; }
	float  GetPressure() { return 0.0f; }
	float  GetTempC() { return 0.0f; }
	AHRS::BoardYawAxis GetBoardYawAxis() { return m_BoardYawAxis; }
	std::string GetFirmwareVersion() { return ""; }

	bool RegisterCallback(ITimestampedDataSubscriber *callback, void *callback_context) { return false; }
	bool DeregisterCallback(ITimestampedDataSubscriber *callback) { return false; }

	int GetActualUpdateRate() { return 0; }
	int GetRequestedUpdateRate() { return 0; }

	void EnableLogging(bool enable) {}

	int16_t GetGyroFullScaleRangeDPS() { return 0; }
	int16_t GetAccelFullScaleRangeG() { return 0; }

private:
	BoardYawAxis m_BoardYawAxis;
	//void SPIInit(SPI::Port spi_port_id, uint32_t bitrate, uint8_t update_rate_hz);
	//void I2CInit(I2C::Port i2c_port_id, uint8_t update_rate_hz);
	//void SerialInit(SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz);
	//void commonInit(uint8_t update_rate_hz);
	//static int ThreadFunc(IIOProvider *io_provider);

	///* SendableBase implementation */
	//void InitSendable(SendableBuilder& builder) override;

	///* PIDSource implementation */
	//double PIDGet();

	//uint8_t GetActualUpdateRateInternal(uint8_t update_rate);

	//nt::NetworkTableEntry m_valueEntry;
};
