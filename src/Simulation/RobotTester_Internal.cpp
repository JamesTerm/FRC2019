#include "pch.h"
#include "Robot_Tester.h"

#include "../main/cpp/Config/ActiveCollection.h"
//#include "../main/cpp/RobotAssem.h"

void SetCommandPromptCallback(std::function<std::string(void)> callback);  //allow this code to manipulate the command prompt
std::string DefaultCommandPrompt();  //set back for proper closing
void RobotControlCommon_SetShowControlsCallback(std::function<bool(void)> callback);
bool RobotControlCommon_ShowControlsDefault();    //set back for proper closing

//This iteration works with the main robot code
#include "../main/cpp/Robot.h"
#pragma comment (lib,"x64/debug/RobotLibraries")
#pragma comment (lib,"x64/debug/TeleopLibraries")

const char * const csz_GameMode_Enum[] =
{
	"Auton","TeleOp","Test"
};

//void SetParentBindCallback(std::function<void(RobotAssem *,bool)> callback);

class RobotTester_Internal
{
private:
	bool m_IsStreaming = false;
	std::future<void> m_TaskState;
	Robot m_Robot;  //what power :)
	enum GameMode
	{
		eAuton,
		eTeleop,
		eTest
	} m_GameMode= eTeleop;  //default is like it is on the driver station
	bool m_DisplayControls = false;
	RobotAssem *m_pRobotAssem=nullptr;  //this is dynamic so we must always check for null
	bool m_HookSamples = false; //cache incase the call is made while robot is not available
	std::function<void(RobotAssem *,bool)> m_ParentBind = nullptr;
public:
	RobotTester_Internal()
	{
		SetCommandPromptCallback(
			[&](void)
		{
			std::string ret = csz_GameMode_Enum[m_GameMode];
			ret += '>';
			return ret;
		});
		RobotControlCommon_SetShowControlsCallback(
			[&](void)
		{
			return m_DisplayControls;
		});
		//SetParentBindCallback(
		//	[&](RobotAssem *instance, bool PropertiesBound)
		//{
		//	//call parent first if available
		//	if (m_ParentBind)
		//		m_ParentBind(instance,PropertiesBound);
		//	if (PropertiesBound)
		//	{
		//		m_pRobotAssem = instance;
		//		if (m_pRobotAssem == nullptr)
		//			m_HookSamples = false;  //unhook if we are being destroyed (pedantic)
		//		HookSampleGoals(m_HookSamples);
		//	}
		//}
		//);
	}
	~RobotTester_Internal()
	{
		SetCommandPromptCallback(DefaultCommandPrompt);
		RobotControlCommon_SetShowControlsCallback(RobotControlCommon_ShowControlsDefault);
		StopStreaming();
	}


	void RobotTester_SetParentBindCallback(std::function<void(RobotAssem *,bool)> callback)
	{
		m_ParentBind = callback;
	}
	void InitRobot()
	{
		m_Robot.RobotInit();
		//Setup callbacks here
		m_Robot.SetIsEnabledCallback(
			[&](void)
			{
				return m_IsStreaming;
			});
		m_Robot.SetIsGameModeCallback(
			[&](void)
			{
				return (int)m_GameMode;  //probably pedantic to cast as an int
			});
	}

	void StartStreaming();
	void StopStreaming()
	{
		if (m_IsStreaming)
		{
			m_IsStreaming = false;
			//join... wait for thread to close:
			size_t TimeOut = 0;
			while (m_TaskState.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready)
			{
				printf("waiting %d\n", TimeOut++);  //put else where so I can monitor both
			}
			//Close any other resources here
		}
	}

	//bool GetIsStreaming() const { return m_IsStreaming; }
	void SetGameMode(int mode)
	{	m_GameMode = (GameMode)mode;
	}
	//Give command ability to switch to different set of goals
	void HookSampleGoals(bool hook=true)
	{	
		#if 0
		if (m_pRobotAssem)
		{
			FRC2019_Robot *_pRobot = dynamic_cast<FRC2019_Robot *>(m_pRobotAssem->GetRobot());
			if (hook)
			{
				_pRobot->SetTestAutonCallbackGoal(
					[](FRC2019_Robot *Robot)
				{
					return FRC2019_Goals::Get_Sample_Goal(Robot);
				});
			}
			else 
				_pRobot->SetTestAutonCallbackGoal(nullptr);
		}
		#endif
	}

	void ShowControls(bool show)
	{
		m_DisplayControls = show;
	}
};


template<typename F, typename... Ts>
inline std::future<typename std::result_of<F(Ts...)>::type> reallyAsync(F&& f, Ts&&... params) // return future for asynchronous call to f(params...)
{
	return std::async(std::launch::async, std::forward<F>(f), std::forward<Ts>(params)...);
}

__inline void MySleep(double Seconds)
{
	const int time_ms = (int)(Seconds*1000.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
}

void RobotTester_Internal::StartStreaming()
{
	if (!m_IsStreaming)
	{
		m_IsStreaming = true;
		//TODO make a reallyAsync that doesn't require an instance
		m_TaskState = reallyAsync(
			[](RobotTester_Internal *Instance)
		{
			printf("starting task_proc()\n");
			switch (Instance->m_GameMode)
			{
			case RobotTester_Internal::eAuton:
				Instance->m_Robot.Autonomous();
				break;
			case RobotTester_Internal::eTeleop:
				Instance->m_Robot.OperatorControl();
				break;
			case RobotTester_Internal::eTest:
				Instance->m_Robot.Test();
				break;
			}
			printf("ending task_proc()\n");
		}
		, this);
	}
}

void RobotTester::RobotTester_create(void)
{
	m_p_RobotTester = std::make_shared<RobotTester_Internal>();
}

void RobotTester::RobotTester_init()
{
	m_p_RobotTester->InitRobot();  //go ahead and init
}


void RobotTester::Test(int test)
{
	m_p_RobotTester->StopStreaming();  //give ability to test consecutively while being able to switch between goal sets
	SetGameMode(2);
	m_p_RobotTester->HookSampleGoals(test==1);
	m_p_RobotTester->StartStreaming();
}

void RobotTester::SetGameMode(int mode)
{
	m_p_RobotTester->SetGameMode(mode);
}

void RobotTester::StartStreaming()
{
	m_p_RobotTester->StartStreaming();
}
void RobotTester::StopStreaming()
{
	m_p_RobotTester->StopStreaming();
}

void RobotTester::ShowControls(bool show)
{
	m_p_RobotTester->ShowControls(show);
}

void RobotTester::HookSampleGoals(bool hook)
{
	m_p_RobotTester->HookSampleGoals(hook);
}

void RobotTester::RobotTester_SetParentBindCallback(std::function<void(RobotAssem *,bool)> callback)
{
	m_p_RobotTester->RobotTester_SetParentBindCallback(callback);
}
