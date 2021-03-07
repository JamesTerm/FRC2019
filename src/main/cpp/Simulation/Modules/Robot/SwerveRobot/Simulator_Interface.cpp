//Avoid the nasty compiler warning by only including what we need
#include <frc/RobotBase.h>
#include <frc/PWMVictorSPX.h>
#include <frc/Encoder.h>
#include <frc/AnalogGyro.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <hal/SimDevice.h>
#include <wpi/math>
#include "../../../../Components/SparkMaxItem.h"
#include "../../../../Components/TalonSRXItem.h"
#include "../../../../Config/ActiveCollection.h"
//use properties for encoder reading conversions
#include "../../../Properties/RegistryV1.h"
//Used for fake run reads of encoder and gyro
#include <networktables/NetworkTableInstance.h>

#include "Simulator_Interface.h"
#include "../DriveKinematics/Vehicle_Drive.h"
#include "../../../Properties/script_loader.h"
#include "SimulatedOdometry.h"

namespace Module {

namespace Output
{

inline double NormalizeRotation3(double Rotation)
{
    //we should really push a SmartDashboard check-box to disable drive
    assert(fabs(Rotation) < Pi2 * 60); //should be less than 60 turns!  If it's greater something is terribly wrong!
    //const double Pi2 = M_PI * 2.0;
    //Normalize the rotation
    while (Rotation > M_PI)
        Rotation -= Pi2;
    while (Rotation < -M_PI)
        Rotation += Pi2;
    return Rotation;
}

//The implementation here is direct and works with the simulation
class WheelModule_Interface
{
private:
    Robot::SwerveVelocities m_PhysicalOdometry;
    std::function<Robot::SwerveVelocities ()> m_VoltageCallback;
    std::function<Robot::SwerveVelocities ()> m_OurSimCallback;
    class WheelModules
    {
    private:
        enum SectionOrder
        {
            eFrontLeft,
            eFrontRight,
            eRearLeft,
            eRearRight
        };
        //we'll keep the same coding conventions as the example
        class WheelModule
        {
        private:
            #pragma region _members_
            Components::SparkMaxItem *m_drive_motor=nullptr;            
            Components::TalonSRXItem *m_swivel_motor=nullptr;
            std::shared_ptr<frc::Encoder> m_driveEncoder=nullptr;  //Note... need two channels per encoder
            std::shared_ptr<frc::Encoder> m_turningEncoder=nullptr;
            //These are not instantiated in the real robot
            std::shared_ptr<frc::sim::EncoderSim> m_driveEncoder_sim=nullptr;
            std::shared_ptr<frc::sim::EncoderSim> m_turningEncoder_sim=nullptr;
            size_t m_ThisSectionIndex;  //see section order (mostly used for diagnostics)
            double m_TalonDPP=1.0;
            double m_SparkDPP=1.0;
            double m_DrivePosition=0.0; //need position reading for some callers which use position
            std::shared_ptr<NetworkTable> m_OutputTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
            std::string m_WheelName;  //for smartdashboard
            std::string m_SwivelName;  //   . . .

            #pragma region _EncoderTranslation_
            class EncoderTranslation_Direct
            {
                #pragma region _Description_
                //This class helps separate the conversion necessary between the encoder readings we get and
                //the readings we need for the odometry, this class sets up the environment directly so that
                //the simulation can use it as a base line, then we derive from it with the actual conversions
                //so both cases can be tested
                #pragma endregion
            protected:
                const Framework::Base::asset_manager *m_props;
                __inline double GetDistancePerPulse_Default() const
                {
                    //keeping this property free for this class
                    //Distance per pulse has to be set properly for real encoder to give radians, the encoder resolution
                    //represents how many pulses per full turn, or in angles it depends on how the vendor publishes it
                    //WPI's example assumes it is for a full turn where 2 pi converts to radians
                    const double kEncoderResolution = 4096.0;
                    return Pi2 / kEncoderResolution;
                }
            public:
                void Init(const Framework::Base::asset_manager *props)
                {
                    m_props=props;
                }
                virtual double Drive_GetDistancePerPulse(size_t wheelSection) const
                {
                    return GetDistancePerPulse_Default();
                }
                virtual double Swivel_GetDistancePerPulse(size_t wheelSection) const
                {
                    return GetDistancePerPulse_Default();
                }
                virtual double Drive_ReadEncoderToLinearVelocity(double encoderReading,size_t wheelSection) const
                {
                    //Linear Velocity is in meters per second, encoder reading is in radians
                    return encoderReading;
                }
                virtual double Drive_SimLinearVelocityToEncoderWrite(double linearVelocity,size_t wheelSection) const
                {
                    //For simulation only and is the inverse computation of the read
                    return linearVelocity;
                }
                virtual double Swivel_ReadEncoderToPosition(double encoderReading,size_t wheelSection) const
                {
                    //Both encoder reading and position are in radians, position is normalized from -pi to pi
                    //it can work within reason from -2pi to 2pi, but should not exceed this
                    //encoder reading has no normalization, but shouldn't be to far off, it starts with zero and can be subjected to
                    //multiple turns in any direction, typically this shouldn't be more than 5 turns so a while loop on the normalization
                    //is fine but to be safe I'd set a limit and trigger a drive disable if it is exceeded, if this fails driving on
                    //a faulty turned wheel can damage robot, so this would be the correct course of action.
                    //The encoder's radians can have the gear reduction in it, but it may be easier to keep like the example, so that 
                    //encoder can be tested by hand, and apply gear reduction here.
                    return encoderReading;
                }
                virtual double Swivel_SimPositionToEncoderWrite(double position,size_t wheelSection) const
                {
                    return position;
                }
            };
            class EncoderTranslation : public EncoderTranslation_Direct
            {
            protected:
                #define GetPropertyName(x)\
                    using namespace properties::registry_v1;\
                    std::string Name = GetPrefix(wheelSection, IsSwivel);\
                    std::string CommonName = GetCommonPrefix(IsSwivel); \
                    Name += #x, CommonName += #x;

                const char* GetPrefix(size_t index, bool IsSwivel) const
                {
                    using namespace properties::registry_v1;
                    //form our prefix, it will use the naming convention in Vehicle Drive.h
                    const char* const prefix_table[2][4] =
                    {
                        {csz_sFL_,csz_sFR_,csz_sRL_,csz_sRR_},
                        {csz_aFL_,csz_aFR_,csz_aRL_,csz_aRR_}
                    };
                    assert(index < 4);
                    const char* const prefix = prefix_table[IsSwivel ? 1 : 0][index];
                    return prefix;
                }
                const char* GetCommonPrefix(bool IsSwivel) const
                {
                    using namespace properties::registry_v1;
                    const char* prefix = IsSwivel ? csz_CommonSwivel_ : csz_CommonDrive_;
                    return prefix;
                }
                double GetDistancePerPulse(size_t wheelSection, bool IsSwivel) const
                {
                    const double kEncoderResolution = 4096.0 / 4;
                    GetPropertyName(Rotary_EncoderPulsesPerRevolution);
                    //This has the common nested as the default, so basically the individual property will return if present
                    //then the common property, then finally the default constant
                    const double enc_resolution= m_props->get_number(Name.c_str(), m_props->get_number(CommonName.c_str(),kEncoderResolution));
                    //I've had a difficult time making sure this was correct... the most important thing to keep in mind is that I work in
                    //radians, so using this on the simulation side should yield the correct amount of ticks that would be there physically
                    //and that is the most important number to get right
                    return Pi2 / enc_resolution;
                }
                double GetGearReduction(size_t wheelSection,bool IsSwivel) const
                {
                    GetPropertyName(Rotary_EncoderToRS_Ratio);
                    //Gear reduction driving over driven (e.g. driven is the bigger gear)
                    return m_props->get_number(Name.c_str(), m_props->get_number(CommonName.c_str(), 1.0));
                }
                double GetWheelRadius() const
                {
                    using namespace properties::registry_v1;
                    return Inches2Meters(m_props->get_number(csz_Drive_WheelDiameter_in,4.0)) * 0.5;
                }
            public:
                bool GetIsReversed(size_t wheelSection, bool IsSwivel) const
                {
                    // returns -1 if reversed
                    GetPropertyName(Rotary_EncoderToRS_Ratio);
                    //Gear reduction driving over driven (e.g. driven is the bigger gear)
                    return m_props->get_bool(Name.c_str(), m_props->get_bool(CommonName.c_str(), false));
                }
                virtual double Drive_GetDistancePerPulse(size_t wheelSection) const
                {
                    return GetDistancePerPulse(wheelSection,false);
                }
                virtual double Swivel_GetDistancePerPulse(size_t wheelSection) const
                {
                    return GetDistancePerPulse(wheelSection, true);
                }
                //TODO: we may want to add Kalman filter and averager for the encoder position, but for now I'll leave it
                //as it probably will not be too noisy given its reduction (unlike a potentiometer)
                virtual double Drive_ReadEncoderToLinearVelocity(double encoderReading,size_t wheelSection) const
                {
                    //Since we can assume radians, we just need to factor in the gear reduction and wheel radius (in meters)
                    return encoderReading * GetGearReduction(wheelSection,false) * GetWheelRadius();
                }
                virtual double Drive_SimLinearVelocityToEncoderWrite(double linearVelocity,size_t wheelSection) const
                {
                    //Factor in the reciprocal radius (1/radius is like divide) and reciprocal gear reduction
                    //Note: Even though compilers are smart these days, to multiply makes it easy to ensure no
                    //division of zero
                    return linearVelocity * (1.0/GetGearReduction(wheelSection, false)) * (1.0/GetWheelRadius());
                }
                virtual double Swivel_ReadEncoderToPosition(double encoderReading,size_t wheelSection) const
                {
                    //Factor in the gear reduction and normalize, for now I'll just assert() the limit
                    return  NormalizeRotation3(encoderReading * GetGearReduction(wheelSection,true));
                }
                virtual double Swivel_SimPositionToEncoderWrite(double position,size_t wheelSection) const
                {
                    //Factor inverse gear reduction, we are normalized so we needn't worry about this
                    return position * (1.0 / GetGearReduction(wheelSection, true));
                }
            };
            #pragma endregion

            EncoderTranslation m_Converter;
            #pragma region _Simulation Variables_
            //keep as pointers to assign during init, because of this we can have both simulation and actual controllers
            //same for encoders, this way we can somewhat test the actual controllers in simulation, and for the actual
            //robot they just remain as null pointers that are never used.

            //This is a fall back as I'll try to use SparkMax in simulation
            std::shared_ptr<frc::PWMVictorSPX> m_sim_drive_motor=nullptr;
            std::shared_ptr<frc::PWMVictorSPX> m_sim_swivel_motor=nullptr;
            std::shared_ptr<frc::sim::SimDeviceSim> m_SparkMaxSimDevice;
            //Testing, for simulation this can bypass the swivel encoders
            double m_TestSwivelPos=0.0;
            #pragma endregion
            #pragma endregion
            bool UseFallbackSim() const
            {
                //override to test actual controllers in the simulation (will need to be open loop, unless we can get vendors to work properly)
                //ultimately, the vendors code should simulate properly, I will keep checking for updates.
                #if 0
                    return frc::RobotBase::IsSimulation();
                #else
                    return false;
                #endif
            }
        public:
            void Init(size_t index,const Framework::Base::asset_manager *props, Configuration::ActiveCollection *collection)
            {
                m_ThisSectionIndex=index;
                m_Converter.Init(props); //This must happen before getting the distance per pulse below
                using namespace frc;
                //Note here we can use the asset manager to switch motor assignments
                //the index is always the section order, but the motor to use can
                //be the property that section order represents
                if (UseFallbackSim())
                {
                    m_sim_drive_motor=std::make_shared<PWMVictorSPX>(index);
                    m_sim_swivel_motor=std::make_shared<PWMVictorSPX>(index+4);
                }
                else
                {
                    //These are hard coded in backupConfig.cpp so we'll just hard codde them here
                    const char *const module_prefix[4]={"FL","FR","BL","BR"};
                    //We do not have access to the actual type of object on sparks so we'll dynamic cast for it
                    std::string name="Wheel";
                    name+=module_prefix[index];
                    m_WheelName=name;
                    Components::NativeComponent *nc=collection->Get(name);
                    assert(nc);
                    m_drive_motor=dynamic_cast<Components::SparkMaxItem *>(nc);
                    assert(m_drive_motor);
                    name ="Swivel";
                    name+=module_prefix[index];
                    m_SwivelName=name;
                    m_swivel_motor=collection->GetTalon(name);
                    assert(m_swivel_motor);
                }

                if (UseFallbackSim())
                {
                    m_driveEncoder=std::make_shared<Encoder>(index*2,index*2+1);
                    m_turningEncoder=std::make_shared<Encoder>((index+4)*2,(index+4)*2+1);
                }

                //Grab our distance per pulse
                if (UseFallbackSim())
                {
                    m_driveEncoder->SetDistancePerPulse(m_Converter.Drive_GetDistancePerPulse(m_ThisSectionIndex));
                    m_turningEncoder->SetDistancePerPulse(m_Converter.Swivel_GetDistancePerPulse(m_ThisSectionIndex));
                }
                else
                {
                    //Note: Leave out setting the DPP, as this we cannot manage it here... instead just cache for simulation
                    //m_drive_motor->SetEncoderRev(m_Converter.Drive_GetDistancePerPulse(m_ThisSectionIndex));
                    //m_swivel_motor->SetDistancePerPulse(m_Converter.Swivel_GetDistancePerPulse(m_ThisSectionIndex));
                    m_SparkDPP=m_Converter.Drive_GetDistancePerPulse(m_ThisSectionIndex);
                    m_TalonDPP=m_Converter.Swivel_GetDistancePerPulse(m_ThisSectionIndex);
                }
                //Grab if we need to reverse direction (broken up to step through code)
                //TODO provide reverse direction for SparkMax and TalonSRX
                if (UseFallbackSim())
                {
                    bool IsReversed=m_Converter.GetIsReversed(m_ThisSectionIndex,false);
                    m_driveEncoder->SetReverseDirection(IsReversed);
                    IsReversed=m_Converter.GetIsReversed(m_ThisSectionIndex,true);
                    m_turningEncoder->SetReverseDirection(IsReversed);
                }

                //Only instantiate if we are in a simulation
                if (RobotBase::IsSimulation())
                {
                    if (UseFallbackSim())
                    {
                        m_driveEncoder_sim=std::make_shared<sim::EncoderSim>(*m_driveEncoder);
                        m_turningEncoder_sim=std::make_shared<sim::EncoderSim>(*m_turningEncoder);
                    }
                    std::string deviceKey = "SPARK MAX [" ;
                    deviceKey += std::to_string(m_ThisSectionIndex);
                    deviceKey += "]";
                    m_SparkMaxSimDevice = std::make_shared<sim::SimDeviceSim>(deviceKey.c_str());
                }
            }
            //This is managed elsewhere, keeping for reference
            void TimeSlice(double dTime_s, double drive_voltage, double swivel_voltage, Robot::SwerveVelocities &physicalOdometry)
            {
                if (UseFallbackSim())
                {
                    m_sim_drive_motor->Set(drive_voltage);
                    m_sim_swivel_motor->Set(swivel_voltage);
                }
                else
                {
                    m_drive_motor->Set(drive_voltage);
                    if (frc::RobotBase::IsSimulation())
                    {
                        hal::SimDouble outputProp = m_SparkMaxSimDevice->GetDouble("Applied Output");
                        outputProp.Set(drive_voltage);
                    }
                    m_swivel_motor->Set(swivel_voltage);
                }
                
                //now to update our odometry
                if (UseFallbackSim())
                {
                    physicalOdometry.Velocity.AsArray[m_ThisSectionIndex]=
                        m_Converter.Drive_ReadEncoderToLinearVelocity(m_driveEncoder->GetRate(),m_ThisSectionIndex);
                    physicalOdometry.Velocity.AsArray[m_ThisSectionIndex+4]=
                        m_Converter.Swivel_ReadEncoderToPosition(m_turningEncoder->GetDistance(),m_ThisSectionIndex);
                }
                #if 0
                //For reference... the motors methods would need to be altered
                else
                {
                    physicalOdometry.Velocity.AsArray[m_ThisSectionIndex]=
                        m_Converter.Drive_ReadEncoderToLinearVelocity(m_drive_motor->GetEncoderVelocity(),m_ThisSectionIndex);

                    #if 0
                    if (m_ThisSectionIndex==0)
                    {
                        const double encoderPos_raw=m_swivel_motor->GetEncoderPosition();
                        const double encoderPos=m_Converter.Swivel_ReadEncoderToPosition(m_swivel_motor->GetEncoderPosition(),m_ThisSectionIndex);
                        printf("-r-%.2f--\n",encoderPos_raw);
                    }
                    #endif
                    //Note:  The config is setup to handle 20ms stresses if we must use talon to transmit encoder readings 
                    //We set the prediction to 20, and add some feed for deceleration on the simulation, may be different for
                    //actual motor.
                    #if 0
                    if (frc::RobotBase::IsReal())
                    {
                        physicalOdometry.Velocity.AsArray[m_ThisSectionIndex+4]=
                            m_Converter.Swivel_ReadEncoderToPosition(m_swivel_motor->GetEncoderPosition(),m_ThisSectionIndex);
                    }
                    else
                        physicalOdometry.Velocity.AsArray[m_ThisSectionIndex+4]=m_TestSwivelPos;
                    #else
                    physicalOdometry.Velocity.AsArray[m_ThisSectionIndex+4]=
                        m_Converter.Swivel_ReadEncoderToPosition(m_swivel_motor->GetEncoderPosition(),m_ThisSectionIndex);
                    #endif
                }
                #endif
            }
            void SimulatorTimeSlice(double dTime_s, double drive_velocity, double swivel_distance) 
            {
                //Note:  this slice does not start until init is complete (solved higher level)
                double driveRate = m_Converter.Drive_SimLinearVelocityToEncoderWrite(drive_velocity, m_ThisSectionIndex);
                double swivelPos=m_Converter.Swivel_SimPositionToEncoderWrite(swivel_distance, m_ThisSectionIndex);
                if (UseFallbackSim())
                {
                    m_driveEncoder_sim->SetRate(driveRate);
                    m_turningEncoder_sim->SetDistance(swivelPos);
                }
                else
                {
                    m_SparkMaxSimDevice->GetDouble("Velocity").Set(driveRate*(1.0/m_SparkDPP));
                    m_swivel_motor->sim_SetQuadratureRawPosition(swivelPos*(1.0/m_TalonDPP));
                    m_TestSwivelPos=swivel_distance;
                   //frc::SmartDashboard::PutNumber("Test",m_drive_motor->GetEncoderVelocity());
                    #if 0
                    if (m_ThisSectionIndex==0)
                    {
                        printf("-s-%.2f--",swivelPos);
                        frc::SmartDashboard::PutNumber("Test",
                        m_Converter.Swivel_ReadEncoderToPosition(m_swivel_motor->GetEncoderPosition(),m_ThisSectionIndex));
                        frc::SmartDashboard::PutNumber("Test2",swivel_distance);
                    }
                    #endif
                }
                //put on the fake run
                m_OutputTable->PutNumber(m_SwivelName + "-Encoder", swivelPos*(1.0/m_TalonDPP));
                m_DrivePosition+= (driveRate*(1.0/m_SparkDPP)) * dTime_s;
                m_OutputTable->PutNumber(m_WheelName + "-Encoder", m_DrivePosition);
            }
            double GetDriveVoltage() const
            {
                return m_drive_motor->Get();
            }
            double GetSwivelVoltage() const
            {
                return m_swivel_motor->Get();
            }
        };

        WheelModule Module[4];
        WheelModule_Interface *m_pParent;
        bool m_StartSimulation = false;
    public:
        WheelModules(WheelModule_Interface *parent) : m_pParent(parent)
        {}
        void Init(const Framework::Base::asset_manager *props, Configuration::ActiveCollection *collection)
        {
            for (size_t i=0;i<4;i++)
                Module[i].Init(i,props,collection);
            m_StartSimulation = true;
        }
        void TimeSlice(double dTime_s)
        {
            //call each module with updated voltages
            for (size_t i=0;i<4;i++)
                Module[i].TimeSlice(dTime_s,
                m_pParent->m_VoltageCallback().Velocity.AsArray[i],
                m_pParent->m_VoltageCallback().Velocity.AsArray[i+4],
                m_pParent->m_PhysicalOdometry);
        }
        void SimulatorTimeSlice(double dTime_s) 
        {
            if (!m_StartSimulation) return;
            for (size_t i=0;i<4;i++)
                Module[i].SimulatorTimeSlice(dTime_s,
                m_pParent->m_OurSimCallback().Velocity.AsArray[i],
                m_pParent->m_OurSimCallback().Velocity.AsArray[i+4]);
        }
        double GetDriveVoltage(size_t index) const
        {
            return Module[index].GetDriveVoltage();
        }
        double GetSwivelVoltage(size_t index) const
        {
            return Module[index].GetSwivelVoltage();
        }

    };
    WheelModules m_WheelModule=this;
public:
    void Init(const Framework::Base::asset_manager *props, Configuration::ActiveCollection *collection)
    {
        m_WheelModule.Init(props,collection);
    }
    void TimeSlice(double dTime_s)
    {
        m_WheelModule.TimeSlice(dTime_s);
    }
    void SimulatorTimeSlice(double dTime_s) 
    {
        m_WheelModule.SimulatorTimeSlice(dTime_s);
    }
    void SetSimOdometry(std::function<Robot::SwerveVelocities ()> callback)
    {
        m_OurSimCallback=callback;
    }
	void SetVoltageCallback(std::function<Robot::SwerveVelocities ()> callback)
    {
        m_VoltageCallback=callback;
    }
	const Robot::SwerveVelocities &GetCurrentVelocities() const
    {
        return m_PhysicalOdometry;
    }
    double GetDriveVoltage(size_t index) const
    {
        return m_WheelModule.GetDriveVoltage(index);
    }
    double GetSwivelVoltage(size_t index) const
    {
        return m_WheelModule.GetSwivelVoltage(index);
    }
};

class WPI_Output_Internal
{
private:
    WheelModule_Interface m_Implementation;
    std::shared_ptr<frc::AnalogGyro> m_Gyro;
    std::shared_ptr<frc::sim::AnalogGyroSim> m_SimGyro;
    std::function<double ()> m_OurSimGyroCallback=nullptr;
    std::shared_ptr<NetworkTable> m_OutputTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
public:
    void Init(const Framework::Base::asset_manager *props, Configuration::ActiveCollection *collection)
    {
        m_Implementation.Init(props,collection);
   		using namespace ::properties::registry_v1;
		const bool HaveGyro = props ? props->get_bool(csz_Misc_have_gyro, false) : false;
        if (HaveGyro)
        {
            using namespace frc;
            const int channel=0;
            //TODO use analog gyro for now since the Nav-x is using fake run, but switch to Nav-x at some point for proper simulation
            m_Gyro=std::make_shared<frc::AnalogGyro>(channel);
            if (RobotBase::IsSimulation())
                m_SimGyro = std::make_shared<sim::AnalogGyroSim>(*m_Gyro);
        }
    }
    void TimeSlice(double dTime_s)
    {
        m_Implementation.TimeSlice(dTime_s);
    }
    void SimulatorTimeSlice(double dTime_s) 
    {
        m_Implementation.SimulatorTimeSlice(dTime_s);
        //Note: this should be normalized to 360, but GEMO
        if ((m_Gyro)&&(m_OurSimGyroCallback))
        {
            const double angle_deg=RAD_2_DEG(m_OurSimGyroCallback());
            m_SimGyro->SetAngle(angle_deg);
            //fake run uses this
            m_OutputTable->PutNumber("NavX-Y", angle_deg);
        }
    }
    void SetSimOdometry(std::function<Robot::SwerveVelocities ()> callback)
    {
        m_Implementation.SetSimOdometry(callback);
    }
    void SetSimOdometry_heading(std::function<double ()> callback)
    {
        m_OurSimGyroCallback=callback;
    }
	void SetVoltageCallback(std::function<Robot::SwerveVelocities ()> callback)
    {
        m_Implementation.SetVoltageCallback(callback);
    }
	const Robot::SwerveVelocities &GetCurrentVelocities() const
    {
        return m_Implementation.GetCurrentVelocities();
    }
    double GyroMag_GetCurrentHeading() const
    {
        //TODO: This is way over-simplified but effective for simulation, here we would access the encoder's predicted position
        //and magnotometer and average them together with some logic for a final heading
        assert(m_Gyro);
        return DEG_2_RAD(m_Gyro->GetAngle());
    }
    double GetDriveVoltage(size_t index) const
    {
        return m_Implementation.GetDriveVoltage(index);
    }
    double GetSwivelVoltage(size_t index) const
    {
        return m_Implementation.GetSwivelVoltage(index);
    }
};

}

namespace Robot {

class Simulator_Interface_Internal
{
private:
    Framework::Base::asset_manager m_properties;
    properties::script_loader m_script_loader;
    SimulatedOdometry m_Simulation;
   	SwerveVelocities m_Voltage;
    Output::WPI_Output_Internal m_Output;
    Robot::Inv_Swerve_Drive m_ExtractVelocity;
    void SetHooks(bool enable)
	{
        if (enable)
		{
			#define HOOK(x,y,z) \
					x(\
					[&](y)\
					{\
						z;\
					});
            HOOK(m_Simulation.SetVoltageCallback,, return m_Voltage);
            HOOK(m_Output.SetVoltageCallback,, return m_Voltage);
            HOOK(m_Output.SetSimOdometry,,return m_Simulation.GetCurrentVelocities());
            HOOK(m_Output.SetSimOdometry_heading,,return m_Simulation.GyroMag_GetCurrentHeading());
   			//done with these macros
			#undef HOOK
        }
        else
		{
			m_Simulation.SetVoltageCallback(nullptr);
        }
    }

	void UpdateVariables()
	{
		using namespace frc;
        using namespace Framework::Base;
        m_ExtractVelocity.InterpolateVelocities(m_Simulation.GetCurrentVelocities());
		Vec2D linear_velocity(LocalToGlobal(m_Simulation.GyroMag_GetCurrentHeading(),
            Vec2D(m_ExtractVelocity.GetLocalVelocityX(),m_ExtractVelocity.GetLocalVelocityY())));
		Vec2D velocity_normalized=linear_velocity;
		double magnitude = velocity_normalized.normalize();
		SmartDashboard::PutNumber("linear_velocity_x", linear_velocity.x());
		SmartDashboard::PutNumber("linear_velocity_y", linear_velocity.y());
		//Entity variables-------------------------------------------
		SmartDashboard::PutNumber("Velocity", Meters2Feet(magnitude));
		SmartDashboard::PutNumber("Rotation Velocity", m_ExtractVelocity.GetAngularVelocity());
		const Vec2D position = m_Simulation.Vision_GetCurrentPosition();
		SmartDashboard::PutNumber("X_ft", Meters2Feet(position.x()));
		SmartDashboard::PutNumber("Y_ft", Meters2Feet(position.y()));
		//If it is angle acceleration is being setpoint driven we read this (e.g. AI controlled)
		if (false)
		{
			//m_current_state.bits.IntendedOrientation = m_robot.Get_IntendedOrientation();
			//SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(m_robot.Get_IntendedOrientation()));
		}
		else
		{
			//TeleOp controlled, point forward if we are rotating in place, otherwise, point to the direction of travel
			//Like with the kinematics if we are not moving we do not update the intended orientation (using this)
			//This is just cosmetic, but may be handy to keep for teleop
			if (!IsZero(linear_velocity.x() + linear_velocity.y(), 0.02))
			{
				//m_current_state.bits.IntendedOrientation = atan2(velocity_normalized[0], velocity_normalized[1]);
				//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
				SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(atan2(velocity_normalized[0], velocity_normalized[1])));
			}
			else if (!IsZero(m_ExtractVelocity.GetAngularVelocity()))
			{
				//point forward locally when rotating in place
				//m_current_state.bits.IntendedOrientation = entity.GetCurrentHeading();
				SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(m_Simulation.GyroMag_GetCurrentHeading()));
			}
		}
		SmartDashboard::PutNumber("Heading", RAD_2_DEG(m_Simulation.GyroMag_GetCurrentHeading()));
		//To make this interesting, we keep the SmartDashboard to show the intended velocities...
		//SmartDashboard::PutNumber("setpoint_angle", RAD_2_DEG(entity.Get_IntendedOrientation()));
		//kinematic variables-------------------------------------------
		const Module::Robot::SwerveVelocities &cv = m_Simulation.GetCurrentVelocities();
		//const Module::Robot::SwerveVelocities &iv = m_robot.GetIntendedVelocities();
		const Module::Robot::SwerveVelocities &vo = m_Voltage;

		// SmartDashboard::PutNumber("Wheel_fl_Velocity", Meters2Feet(iv.Velocity.AsArray[0]));
		// SmartDashboard::PutNumber("Wheel_fr_Velocity", Meters2Feet(iv.Velocity.AsArray[1]));
		// SmartDashboard::PutNumber("Wheel_rl_Velocity", Meters2Feet(iv.Velocity.AsArray[2]));
		// SmartDashboard::PutNumber("Wheel_rr_Velocity", Meters2Feet(iv.Velocity.AsArray[3]));
		SmartDashboard::PutNumber("Wheel_fl_Voltage", vo.Velocity.AsArray[0]);
		SmartDashboard::PutNumber("Wheel_fr_Voltage", vo.Velocity.AsArray[1]);
		SmartDashboard::PutNumber("Wheel_rl_Voltage", vo.Velocity.AsArray[2]);
		SmartDashboard::PutNumber("Wheel_rr_Voltage", vo.Velocity.AsArray[3]);
		SmartDashboard::PutNumber("wheel_fl_Encoder", Meters2Feet(cv.Velocity.AsArray[0]));
		SmartDashboard::PutNumber("wheel_fr_Encoder", Meters2Feet(cv.Velocity.AsArray[1]));
		SmartDashboard::PutNumber("wheel_rl_Encoder", Meters2Feet(cv.Velocity.AsArray[2]));
		SmartDashboard::PutNumber("wheel_rr_Encoder", Meters2Feet(cv.Velocity.AsArray[3]));

		//For the angles either show raw or use simple dial using 180 to -180 with a 45 tick interval
		//its not perfect, but it gives a good enough direction to tell (especially when going down)
		SmartDashboard::PutNumber("Swivel_fl_Voltage", vo.Velocity.AsArray[4]);
		SmartDashboard::PutNumber("Swivel_fr_Voltage", vo.Velocity.AsArray[5]);
		SmartDashboard::PutNumber("Swivel_rl_Voltage", vo.Velocity.AsArray[6]);
		SmartDashboard::PutNumber("Swivel_rr_Voltage", vo.Velocity.AsArray[7]);
		SmartDashboard::PutNumber("swivel_fl_Raw", RAD_2_DEG(cv.Velocity.AsArray[4]));
		SmartDashboard::PutNumber("swivel_fr_Raw", RAD_2_DEG(cv.Velocity.AsArray[5]));
		SmartDashboard::PutNumber("swivel_rl_Raw", RAD_2_DEG(cv.Velocity.AsArray[6]));
		SmartDashboard::PutNumber("swivel_rr_Raw", RAD_2_DEG(cv.Velocity.AsArray[7]));
	}

public:
    Simulator_Interface_Internal()
    {
        SetHooks(true);
    }
   	void Shutdown()
	{
		SetHooks(false);
	}
	~Simulator_Interface_Internal()
	{
		Shutdown();
	}
	void Reset()
	{
		m_Voltage = {};
	}
    void SimulationInit()
    {
        //pass the properties into the simulation
        m_Simulation.Init(&m_properties);
        Reset();
    }
    void ActiveCollection_Init(Configuration::ActiveCollection *collection)
    {
        //Note... it turns out this is called before Simulation, but this should be more robust to handle either case
   		m_script_loader.load_script(m_properties); 
        m_Output.Init(&m_properties,collection);
    }
    void TimeSlice(double dTime_s)
    {
        const bool can_run=nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetBoolean("RUN_ROBOT", false);
        if (!can_run)
            return;
        //Grab the voltage from motors (they are being set externally)
        for (size_t i=0;i<4;i++)
        {
            m_Voltage.Velocity.AsArray[i]=m_Output.GetDriveVoltage(i);
            m_Voltage.Velocity.AsArray[i+4]=m_Output.GetSwivelVoltage(i);
        }
   		//The simulation already is hooked to m_Voltage its ready to simulate
		//This call is skipped in real robot code as it physically happens instead
        m_Simulation.TimeSlice(dTime_s);
        m_Output.SimulatorTimeSlice(dTime_s);
        UpdateVariables();
    }
};

#pragma region _wrapper methods_

void Simulator_Interface::SimulationInit()
{
    assert(m_interface);
    m_interface->SimulationInit();
}
void Simulator_Interface::ActiveCollection_Init(void *active_collection)
{
    //Note... it turns out this is called before Simulation, but this should be more robust to handle either case
    m_interface = std::make_shared<Simulator_Interface_Internal>();
    m_interface->ActiveCollection_Init((Configuration::ActiveCollection *)active_collection);
}
void Simulator_Interface::TimeSlice(double dTime_s)
{
    m_interface->TimeSlice(dTime_s);
}

#pragma endregion
    }
}