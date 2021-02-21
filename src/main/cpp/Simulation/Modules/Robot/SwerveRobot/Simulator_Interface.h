#pragma once
#include <memory>
#include <functional>

#include <memory>
#include <functional>
namespace Module {
	namespace Robot {

class Simulator_Interface_Internal;
class Simulator_Interface
{
    #pragma region _description_
    //Here is a method friendly interface that brings together the simulation
    #pragma endregion
public:
    //This is exclusive for SimulationInit() to call, anything in here will not be called on the real robot
    void SimulationInit();
    //This is exclusively for SimulationPeriodic() to call, anything in here will not be called on the real robot
    void TimeSlice(double dTime_s);
private:
    std::shared_ptr<Simulator_Interface_Internal> m_interface;
};
    }
}