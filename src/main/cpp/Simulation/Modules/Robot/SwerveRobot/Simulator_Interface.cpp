#include "Simulator_Interface.h"
#include "../DriveKinematics/Vehicle_Drive.h"
#include "../../../Properties/script_loader.h"
#include "SimulatedOdometry.h"

namespace Module {
	namespace Robot {

class Simulator_Interface_Internal
{
private:
    Framework::Base::asset_manager m_properties;
    properties::script_loader m_script_loader;
    SimulatedOdometry m_Simulation;
   	SwerveVelocities m_Voltage;
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
   			//done with these macros
			#undef HOOK
        }
        else
		{
			m_Simulation.SetVoltageCallback(nullptr);
        }
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
   		m_script_loader.load_script(m_properties);
        m_Simulation.Init(&m_properties);
        Reset();
    }
    void TimeSlice(double dTime_s)
    {
   		//The simulation already is hooked to m_Voltage its ready to simulate
		//This call is skipped in real robot code as it physically happens instead
        m_Simulation.TimeSlice(dTime_s);
    }
};

#pragma region _wrapper methods_

void Simulator_Interface::SimulationInit()
{
    m_interface = std::make_shared<Simulator_Interface_Internal>();
    m_interface->SimulationInit();
}

void Simulator_Interface::TimeSlice(double dTime_s)
{
    m_interface->TimeSlice(dTime_s);
}

#pragma endregion
    }
}