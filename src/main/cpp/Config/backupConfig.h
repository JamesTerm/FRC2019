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
#include "../Pugi/pugixml.h"
#include "../Goals/GoalSelector.h"
#include "../Util/Log.h"
#include "../Limelight/limelight.h"
#include "../Components/SparkMaxItem.h"
#include "../Components/REVColorSensorV3.h"
#include "Config.h"

using namespace System;
using namespace pugi;
using namespace Lime;
namespace Configuration
{
    class backupConfig {
public:
	backupConfig(ActiveCollection *_activeCollection, Drive *_drive);

	virtual ~backupConfig();
private:
    bool AtCompVal = false;
    double Version = 0.1;
    bool useNavX = false;
	bool useLimelight = false;
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

    void AddSparkMax(string Name, int Channel, bool Reversed){SparkMaxItem *tmp = new SparkMaxItem(Channel, Name, Reversed);
			    	m_activeCollection->Add(tmp);
                    Log::General("Added Spark Max " + Name + ", Channel: " + to_string(Channel) + ", Reversed: " + to_string(Reversed));};

    void AddTalonSRX(string Name, int Channel, bool Reversed, bool enableEncoder){TalonSRXItem *tmp = new TalonSRXItem(Channel, Name, Reversed, enableEncoder);
			    	m_activeCollection->Add(tmp);
                    Log::General("Added TalonSRX " + Name + ", Channel: " + to_string(Channel) + ", Reversed: " + to_string(Reversed) + ", encoder: " + to_string(enableEncoder));};

    void AddPotentiometer(string Name, int Channel){PotentiometerItem *tmp = new PotentiometerItem(Channel, Name);
				    m_activeCollection->Add(tmp);
                    Log::General("Added Potentiometer " + Name + ", Channel: " + to_string(Channel));};

    void AddEncoder(string Name, int ChannelA, int ChannelB, bool reversed){EncoderItem *tmp = new EncoderItem(Name, ChannelA, ChannelB, reversed);
				    m_activeCollection->Add(tmp);
                    Log::General("Added Encoder " + Name + ", A-Channel: " + to_string(ChannelA) + ", B-Channel: " + to_string(ChannelB) + ", Reversed: " + to_string(reversed));};

    void AddDoubleSolenoid(string Name, int fChannel, int rChannel, DoubleSolenoid::Value _def, bool reversed){DoubleSolenoidItem *tmp = new DoubleSolenoidItem(Name , fChannel, rChannel, _def, reversed);
			    	m_activeCollection->Add(tmp);
			    	Log::General("Added DoubleSolenoid " + Name + ", F-Channel: " + to_string(fChannel) + ", R-Channel: " + to_string(rChannel) + ", Default: " + to_string(_def) + ", Reversed: " + to_string(reversed));};

    void AddDigitalInput(string Name, int Channel){DigitalInputItem *tmp = new DigitalInputItem(Channel, Name);
			    	m_activeCollection->Add(tmp);
			    	Log::General("Added DigitalInput " + Name + ", Channel: " + to_string(Channel));};


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

	void AddGoalControl(string name, TeleOpGoal goalToAdd, int channel, double params, bool bind_event, JoystickControler Person)
	{
		GoalButtonControl* tmp = new GoalButtonControl((Person == JoystickControler::Driver ? m_driveJoy : m_operatorJoy), name, channel, m_activeCollection, goalToAdd, params);
		if (Person == JoystickControler::Driver)
			m_drive->AddControlDrive(tmp);
		else
			m_drive->AddControlOperate(tmp);
		Log::General("Added GoalButtonControl " + name + ", Button: " + to_string(channel) + ", Goal: " + to_string(goalToAdd) + ", Params: " + to_string(params));
		
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

};
}


#endif /* SRC_BACKUP_CONFIG_H_ */