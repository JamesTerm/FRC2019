/****************************** Header ******************************\
Class Name: backupConfig
File Name:	backupConfig.h
Summary: 	If the config file doesnt exist config loads this instead
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#ifndef SRC_BACKUP_CONFIG_H_
#define SRC_BACKUP_CONFIG_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "ActiveCollection.h"
#include "../Systems/Drive.h"
#include "../Controls/ButtonControl.h"
#include "../Controls/AxisControl.h"
#include "../Controls/ToggleButtonControl.h"
#include "../Controls/GoalButtonControl.h"
#include "../Controls/GoalAxisControl.h"
#include "../Pugi/pugixml.h"
#include "../Goals/GoalSelector.h"
#include "../Util/Log.h"
#include "../Limelight/limelight.h"
#include "../Components/SparkMaxItem.h"
#include "../Components/REVColorSensorV3.h"
#include "../Components/Swerve/SwerveManager.h"
#include "../Components/Swerve/SwerveModule.h"
#include "Config.h"

using namespace System;
using namespace pugi;
using namespace Lime;
using namespace std;
namespace Configuration
{
    class backupConfig {
public:
	backupConfig(ActiveCollection *_activeCollection, Drive *_drive);

	virtual ~backupConfig();
private:
    bool AtCompVal = false;
    double Version = 5.0;
    bool useNavX = true;
	bool useLimelight = false;
	bool Fake = true;
	bool Real = !Fake;
	int DriverSlot = 0;
	int OperatorSlot = 1;

	ActiveCollection *m_activeCollection;
	Drive *m_drive;
	Joystick *m_driveJoy;
	Joystick *m_operatorJoy;

	enum JoystickControler {Driver = 0, Operator = 1};
    
    void SetInfo();
    void SetComponents();
    void SetControls();

    void AddVictor(string Name, int Channel, bool Reversed){VictorSPItem *tmp = new VictorSPItem(Name, Channel, Reversed);
				    m_activeCollection->Add(tmp);
                    Log::General("Added VictorSP " + Name + ", Channel: " + to_string(Channel) + ", Reversed: " + to_string(Reversed));};

    void AddVictorSPX(string Name, int Channel, bool Reversed){VictorSPXItem *tmp = new VictorSPXItem(Channel, Name, Reversed);
				    m_activeCollection->Add(tmp);
                    Log::General("Added VictorSPX " + Name + ", Channel: " + to_string(Channel) + ", Reversed: " + to_string(Reversed));};

    void AddSparkMax(string Name, int Channel, bool Reversed){SparkMaxItem *tmp = new SparkMaxItem(Channel, Name, Reversed, Real);
			    	m_activeCollection->Add(tmp);
                    Log::General("Added Spark Max " + Name + ", Channel: " + to_string(Channel) + ", Reversed: " + to_string(Reversed));};

    void AddTalonSRX(string Name, int Channel, bool Reversed, bool enableEncoder){TalonSRXItem *tmp = new TalonSRXItem(Channel, Name, Reversed, enableEncoder, Real);
			    	m_activeCollection->Add(tmp);
                    Log::General("Added TalonSRX " + Name + ", Channel: " + to_string(Channel) + ", Reversed: " + to_string(Reversed) + ", encoder: " + to_string(enableEncoder));};

    void AddPotentiometer(string Name, int Channel){PotentiometerItem *tmp = new PotentiometerItem(Channel, Name, Real);
				    m_activeCollection->Add(tmp);
                    Log::General("Added Potentiometer " + Name + ", Channel: " + to_string(Channel));};

    void AddEncoder(string Name, int ChannelA, int ChannelB, bool reversed){EncoderItem *tmp = new EncoderItem(Name, ChannelA, ChannelB, reversed, Real);
				    m_activeCollection->Add(tmp);
                    Log::General("Added Encoder " + Name + ", A-Channel: " + to_string(ChannelA) + ", B-Channel: " + to_string(ChannelB) + ", Reversed: " + to_string(reversed));};

    void AddDoubleSolenoid(string Name, int fChannel, int rChannel, DoubleSolenoid::Value _def, bool reversed){DoubleSolenoidItem *tmp = new DoubleSolenoidItem(Name , fChannel, rChannel, _def, reversed, Real);
			    	m_activeCollection->Add(tmp);
			    	Log::General("Added DoubleSolenoid " + Name + ", F-Channel: " + to_string(fChannel) + ", R-Channel: " + to_string(rChannel) + ", Default: " + to_string(_def) + ", Reversed: " + to_string(reversed));};

    void AddDigitalInput(string Name, int Channel){DigitalInputItem *tmp = new DigitalInputItem(Channel, Name, Real);
			    	m_activeCollection->Add(tmp);
			    	Log::General("Added DigitalInput " + Name + ", Channel: " + to_string(Channel));};

	void Link_Motor_Encoder(string MotorName, string EncoderName)
	{
		if (m_activeCollection->GetEncoder(EncoderName) != nullptr)
		{
			if(m_activeCollection->Get(MotorName) != nullptr)
			{
				((Motor*)m_activeCollection->Get(MotorName))->SetLinkedEncoder(m_activeCollection->GetEncoder(EncoderName));
			}
			else
			{
				Log::Error(MotorName + " doesnt exist!");
			}
		}
		else
		{
			Log::Error(EncoderName + " doesnt exist!");
		}
	};

	void AddSwerveModule(string Name, string SwivelMotor, string WheelMotor, double Ticksperrev, double WheelTicksperrev, SwerveModule::Location Loc)
	{
		if (m_activeCollection->Get(SwivelMotor) != nullptr && m_activeCollection->Get(WheelMotor) != nullptr)
		{
			SwerveModule *tmp = new SwerveModule(Name, (Motor*)m_activeCollection->Get(SwivelMotor), (Motor*)m_activeCollection->Get(WheelMotor), Ticksperrev, WheelTicksperrev);
			tmp->SetLocation(Loc);
			m_activeCollection->Add(tmp);
			string Loca = "robot";

			switch(Loc)
                {
                    case  SwerveModule::Location::Front_Left:
                        Loca = "Front Left";
                    break;
                    case  SwerveModule::Location::Front_Right:
                        Loca = "Front Right";
                    break;
                    case  SwerveModule::Location::Back_Left:
                        Loca = "Back Left";
                    break;
                    case  SwerveModule::Location::Back_Right:
                        Loca = "Back Right";
                    break;
                }

			Log::General("Added Swerve Module :" + Name + " in " + Loca);
		}
	};

	void AddSwerveManager(string name, bool Wait, string LeftF, string RightF, string LeftB, string RightB, double _Length, double _Width)
	{
		if (m_activeCollection->Get(LeftF) != nullptr && m_activeCollection->Get(RightF) != nullptr && m_activeCollection->Get(LeftB) != nullptr && m_activeCollection->Get(RightB) != nullptr)
		{
			SwerveManager *Manager = new SwerveManager(name, Wait, (SwerveModule*)m_activeCollection->Get(LeftF), (SwerveModule*)m_activeCollection->Get(RightF), (SwerveModule*)m_activeCollection->Get(LeftB), (SwerveModule*)m_activeCollection->Get(RightB), _Length, _Width);
			m_activeCollection->Add(Manager);
			Log::General("Added Swerve Manager");
		}
	};

	void AddSwerveManager(string name, bool Wait, double Max, string LeftF, string RightF, string LeftB, string RightB, double _Length, double _Width)
	{
		if (m_activeCollection->Get(LeftF) != nullptr && m_activeCollection->Get(RightF) != nullptr && m_activeCollection->Get(LeftB) != nullptr && m_activeCollection->Get(RightB) != nullptr)
		{
			SwerveManager *Manager = new SwerveManager(name, Wait, (SwerveModule*)m_activeCollection->Get(LeftF), (SwerveModule*)m_activeCollection->Get(RightF), (SwerveModule*)m_activeCollection->Get(LeftB), (SwerveModule*)m_activeCollection->Get(RightB), _Length, _Width);
			Manager->SetMaxPow(Max);
			m_activeCollection->Add(Manager);
			Log::General("Added Swerve Manager with a max speed of: " + to_string(Max));
		}
	};

	void AddSwerveManager(string name, bool Wait, double Max, string LeftF, string RightF, string LeftB, string RightB, NavX *Nav, double _Length, double _Width)
	{
		if (m_activeCollection->Get(LeftF) != nullptr && m_activeCollection->Get(RightF) != nullptr && m_activeCollection->Get(LeftB) != nullptr && m_activeCollection->Get(RightB) != nullptr)
		{
			SwerveManager *Manager = new SwerveManager(name, Wait, (SwerveModule*)m_activeCollection->Get(LeftF), (SwerveModule*)m_activeCollection->Get(RightF), (SwerveModule*)m_activeCollection->Get(LeftB), (SwerveModule*)m_activeCollection->Get(RightB), Nav, _Length, _Width);
			Manager->SetMaxPow(Max);
			m_activeCollection->Add(Manager);
			Log::General("Added Swerve Manager with location");
		}
	};

	void AddSwerveManager(string name, bool Wait, double Max, string Modules, NavX *Nav, double _Length, double _Width)
	{
		istringstream ss(Modules);
    	string word;
		vector<SwerveModule*> Parts;
		bool AllHere = true;
    	while (ss >> word) 
    	{
			if(m_activeCollection->Get(word) != nullptr)
			{
				Parts.push_back((SwerveModule*)m_activeCollection->Get(word));
			}
			else
			{
				AllHere = false;
			}
    	}

		if (AllHere)
		{
			SwerveManager *Manager = new SwerveManager(name, Wait, Parts, Nav, _Length, _Width);
			Manager->SetMaxPow(Max);
			m_activeCollection->Add(Manager);
			Log::General("Added Swerve Manager with location");
		}
		else
		{
			Log::Error("One or modules not found!");
		}
	};

	void AddSwerveControl(string name, SwerveControl::DriveCalculation Cal, int H, int V, int S, double dz, double mult, bool reversed, string ManagerName, JoystickControler Person)
	{
		if (m_activeCollection->Get(ManagerName) != nullptr)
		{
			SwerveControl *tmp = new SwerveControl((Person == JoystickControler::Driver ? m_driveJoy : m_operatorJoy), Cal, name, V, H, S, dz, reversed, mult, m_activeCollection, (SwerveManager*)m_activeCollection->Get(ManagerName));
			string Mode = (Cal == SwerveControl::DriveCalculation::Field_Oriented ? "FIELD" : "ROBOT");
			Log::General("Added swerve control that is " + Mode + " oriented");
			if (Person == JoystickControler::Driver)
				m_drive->AddControlDrive(tmp);
			else
				m_drive->AddControlOperate(tmp);
		}
		else
		{
			Log::Error("Swerve control cannot find manager with name " + ManagerName);
		}
	};

    void AddAxisControl(string name, string bind_string, int channel, double multiply, bool reversed, bool useOverdrive, bool bind_event, bool isLift, double deadZone, JoystickControler Person)
    {
		AxisControl *tmp = new AxisControl((Person == JoystickControler::Driver ? m_driveJoy : m_operatorJoy), name, channel, deadZone, reversed, multiply, m_activeCollection, useOverdrive);
		if (Person == JoystickControler::Driver)
			m_drive->AddControlDrive(tmp);
		else
			m_drive->AddControlOperate(tmp);
		Log::General("Added AxisControl " + name + ", Axis: " + to_string(channel) + ", DeadZone: " + to_string(deadZone) + ", Reversed: " + to_string(reversed) + ", Power Multiplier: " + to_string(multiply) + " Is Lift: " + to_string(isLift));
		vector<string> bind_vector = getBindingStringList(bind_string);
		setBindingsToControl(bind_vector, tmp);
		
		if(isLift)
			tmp->SetLift(1.5, m_activeCollection);
		if(bind_event){
			m_activeCollection->AddEvent(&(tmp->ValueChanged));
		}
    };

	void AddButtonControl(string name, string bind_string, int channel, double multiply, bool reversed, bool isOverdrive, bool bind_event, bool isSolinoid, bool actOnRelease, bool isAmpLimited, bool isRamp, JoystickControler Person)
	{
		ButtonControl *tmp = new ButtonControl((Person == JoystickControler::Driver ? m_driveJoy : m_operatorJoy), name, channel, actOnRelease, reversed, multiply, isSolinoid, m_activeCollection, isOverdrive);
		if (Person == JoystickControler::Driver)
			m_drive->AddControlDrive(tmp);
		else
			m_drive->AddControlOperate(tmp);

		string actOnRelease_print = actOnRelease ? "true" : "false";
		string reversed_print = reversed ? "true" : "false";
		string isSolenoid_print = isSolinoid ? "true" : "false";
		string isOverdrive_print = isOverdrive ? "true" : "false";
		Log::General("Added Button Control " + name + ", Button: " + to_string(channel) + ", ActOnRelease: " + actOnRelease_print + ", Reversed: " + reversed_print + ", PowerMultiplier: " + to_string(multiply) + ", IsSolenoid: " + isSolenoid_print + ", IsOverdrive: " + isOverdrive_print);
		
		vector<string> bind_vector = getBindingStringList(bind_string);
		setBindingsToControl(bind_vector, tmp);

		if(isAmpLimited)
			tmp->SetAmpRegulation(11, 30);
		if (isRamp)
			tmp->SetRamp(0.1);
		
		if(bind_event){
			m_activeCollection->AddEvent(&(tmp->ValueChanged));
		}
	};

	void AddToggleControl(string name, string bind_string, int channel, double multiply, bool reversed, bool bind_event, JoystickControler Person)
	{
		ToggleButtonControl *tmp = new ToggleButtonControl((Person == JoystickControler::Driver ? m_driveJoy : m_operatorJoy), name, channel, reversed, multiply, m_activeCollection);
		if (Person == JoystickControler::Driver)
			m_drive->AddControlDrive(tmp);
		else
			m_drive->AddControlOperate(tmp);
		Log::General("Added Toggle Control " + name + ", Button: " + to_string(channel) + ", multiply: " + to_string(multiply) + ", reversed: " + to_string(reversed));
		
		vector<string> bind_vector = getBindingStringList(bind_string);
		setBindingsToControl(bind_vector, tmp);
		
		if(bind_event){
			m_activeCollection->AddEvent(&(tmp->ValueChanged));
		}
	};

	void AddGoalButtonControl(string name, TeleOpGoal goalToAdd, int channel, double params, int KeyID, vector<int> RemoveKeys, bool bind_event, JoystickControler Person)
	{
		GoalButtonControl* tmp = new GoalButtonControl((Person == JoystickControler::Driver ? m_driveJoy : m_operatorJoy), name, channel, m_activeCollection, goalToAdd, params, KeyID, RemoveKeys);
		if (Person == JoystickControler::Driver)
			m_drive->AddControlDrive(tmp);
		else
			m_drive->AddControlOperate(tmp);
		Log::General("Added GoalButtonControl " + name + ", Button: " + to_string(channel) + ", Goal: " + to_string(goalToAdd) + ", Params: " + to_string(params));
		
		if(bind_event){
			m_activeCollection->AddEvent(&(tmp->ValueChanged));
		}
	};

	void AddGoalAxisControl(string name, TeleOpGoal goalToAdd, vector<int> Axis, vector<string> StringData, int KeyID, vector<int> RemoveKeys, int startIndex, bool bind_event, bool RepeatWhenFinished, JoystickControler Person)
	{
		GoalAxisControl* tmp = new GoalAxisControl((Person == JoystickControler::Driver ? m_driveJoy : m_operatorJoy), name, Axis, m_activeCollection, goalToAdd, StringData, startIndex, KeyID, RemoveKeys, RepeatWhenFinished);
		if (Person == JoystickControler::Driver)
			m_drive->AddControlDrive(tmp);
		else
			m_drive->AddControlOperate(tmp);
		Log::General("Added GoalAxisControl " + name + ", Goal: " + to_string(goalToAdd));
		
		if(bind_event){
			m_activeCollection->AddEvent(&(tmp->ValueChanged));
		}
	};

    vector<string> getBindingStringList(string bindings)
    {
	    vector<char*> tmp;
    	vector<string> ret;
	    char * bindings_char = new char[bindings.length() + 1];
	    strcpy(bindings_char, bindings.c_str());
	    tmp.push_back(strtok(bindings_char, " ,"));
	    while(tmp.back() != NULL){
    		tmp.push_back(strtok(NULL, " ,"));				
	    }
	    tmp.pop_back();
	    for(int i = 0; i<(int)tmp.size(); i++){
    		string add(tmp[i]);
		    ret.push_back(add);
	    }
	    return ret;
    }

    bool setBindingsToControl(vector<string> bindings, ControlItem *control)
    {
    	bool success = true;
    	for(int i = 0; i<(int)bindings.size(); i++){
		    string binding = bindings[i];
		    OutputComponent *component;
		    try{
			    component = (OutputComponent*)(m_activeCollection->Get(binding));
			    control->AddComponent(component);
			    Log::General("Allocated Component " + binding + " to Control " + control->name);
		    }
	    	catch(...){
		    	Log::Error("Failed to bind component " + binding + " to the control " + control->name + ". This can cause fatal runtime errors!");
			    success = false;
		    }
	    }
	    if(!success)
    		Log::Error("One or more bindings failed to allocate for control " + control->name + ". Please check the Config!");
    	return success;
    }

	void AddPIDProfile(string Name, double P, double I, double D, double MaxChange = 0.1, double Bias = -1, double Min = -1, double Max = 1, int index = -1)
	{
		int IndexAt = m_activeCollection->CreateAndAddProfile(Name, P, I, D, MaxChange, (Bias <= 0 ? P * 100 : Bias), Min, Max, index);
		Log::General("Added PIDProfile: " + Name + " at index: " + to_string(IndexAt));
	}

	void Link_Motor_PositionProfile(string NameMotor, string NameProfile)
	{
		Motor* MotorPtr = (Motor*)m_activeCollection->Get(NameMotor);
		if(MotorPtr != nullptr)
		{
			MotorPtr->SetPositonProfile(m_activeCollection->GetProfile(NameProfile));
			Log::General("Correctly Set Motor: " + NameMotor + " to PID numbers from: " + NameProfile);
		}
		else
		{
			Log::Error("Motor " + NameMotor + " does not exist!");
		}
	}

	void Link_Motor_PositionProfile(string NameMotor, int IndexProfile)
	{
		Motor* MotorPtr = (Motor*)m_activeCollection->Get(NameMotor);
		if(MotorPtr != nullptr)
		{
			MotorPtr->SetPositonProfile(m_activeCollection->GetProfile(IndexProfile));
			Log::General("Correctly Set Motor: " + NameMotor + " to PID numbers from index: " + to_string(IndexProfile));
		}
		else
		{
			Log::Error("Motor " + NameMotor + " does not exist!");
		}
	}

	void Link_Motor_PowerProfile(string NameMotor, string NameProfile)
	{
		Motor* MotorPtr = (Motor*)m_activeCollection->Get(NameMotor);
		if(MotorPtr != nullptr)
		{
			MotorPtr->SetPowerProfile(m_activeCollection->GetProfile(NameProfile));
			Log::General("Correctly Set Motor: " + NameMotor + " to PID numbers from: " + NameProfile);
		}
		else
		{
			Log::Error("Motor " + NameMotor + " does not exist!");
		}
	}

	void Link_Motor_PowerProfile(string NameMotor, int IndexProfile)
	{
		Motor* MotorPtr = (Motor*)m_activeCollection->Get(NameMotor);
		if(MotorPtr != nullptr)
		{
			MotorPtr->SetPowerProfile(m_activeCollection->GetProfile(IndexProfile));
			Log::General("Correctly Set Motor: " + NameMotor + " to PID numbers from index: " + to_string(IndexProfile));
		}
		else
		{
			Log::Error("Motor " + NameMotor + " does not exist!");
		}
	}

	void SetAutoPath(string Name, bool OverrideDS = true)
	{
		Log::General("Selected Auto: " + Name + ".txt" + (OverrideDS ? " --Override DS" : " --Not Override DS"));
		m_activeCollection->SetAuto(Name + ".txt");
		m_activeCollection->SetAutoOverride(OverrideDS);
	}

};
}


#endif /* SRC_BACKUP_CONFIG_H_ */