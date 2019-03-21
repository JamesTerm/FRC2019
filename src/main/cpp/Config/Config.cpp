/****************************** Header ******************************\
Class Name: Config
File Name:	Config.h
Summary: 	Manages and loads the configuration file.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com
\********************************************************************/

#include "Config.h"
#include <string.h>
#include <stdio.h>

using namespace std;
using namespace System;
using namespace Configuration;
using namespace pugi;
using namespace frc;

/**
 * Load in the document
 * Config Version and Comment (different from the Robot.cpp Version and Comment)
 * * Verbose Output: Need to give Ian the logging TODO first
 * ? AutonEnabled
 * Secondary Camera Server
 * * Vision Initialization -> need vision working with that
 * Allocate Components
 *TODO: Encoders-> Add encoders to motors
 * DI
 *TODO: *DO-> Write the abstraction classes
 *TODO:*AI-> Write the abstraction classes
 *TODO:*AO-> Write the abstraction classes
 *TODO: Lower and Upper Limit for DAI
 * Motors
 *?	Drive class integration? Probably Post-Season
 * Solenoids
 *?	Relays
 * Potentiometers
 * *Allocate the Joysticks via the XML
 * *Allocate Controls (make another method for this, prolly one for each joystick)
 * 
**/ 
Config::Config(ActiveCollection *_activeCollection, Drive *_drive) {
//? make doc a member variable?
	xml_document doc;
	xml_parse_result result = doc.load_file("config.xml");
	m_activeCollection = _activeCollection;
	m_drive = _drive;
	if (result)
	{
		Log::General("XML Config parsed without errors");
		LoadValues(doc);
	}
	else
	{
		//In simulation we should really get the message across
		#ifdef _Win32
		assert(false);  
		#endif
		Log::Error("XML Config parsed with errors");
		string desc = result.description();
		Log::Error("Error description: " + desc);
		Log::Error("Error offset: " + result.offset);
		Log::Error("No config available, returning to Robot.cpp\nTHIS IS A BIG ERROR!");
	}
}

void Config::LoadValues(xml_document &doc){
	xml_node root = doc.child("Robot");
	
	if(root){
		Log::General("Config Root found");
	}
	else{
		Log::General("Robot was not found in Config! I hope you didn't intend to configure anything! Returning to Robot.cpp");
		return;
	}

	#pragma region MetaData

	#pragma region Version

	xml_attribute version = root.child("Version").attribute("version");
	if(version)
		Log::General("Config Version: " + version.as_int(), true);
	else
		Log::Error("Config Version not found");

	#pragma endregion Version

	#pragma region Comment

	xml_attribute comment = root.child("Comment").attribute("comment");
	if(comment){
		string comm = comment.as_string();
		Log::General("Comment: " + comm, true);
	}
	else
		Log::Error("Comment not found");

	#pragma endregion Comment

	#pragma region NavX

	xml_attribute useNavX = root.child("UseNavX").attribute("value");
	//TODO: Addition of other NavX ports
	if(useNavX){
		if(useNavX.as_bool()){
			m_activeCollection->Add(new NavX());
			Log::General("NavX detected");
		}
		else
			Log::General("Failed to load the NavX: Disabling NavX by default");
			
	}
	else
		Log::Error("UseNavX not found. Disabling by default");

	#pragma endregion NavX

	#pragma region SecondaryCameraServer

	//TODO: make it so we can mess with the camera during the running of the robot: ie, switch which stream we are using 
	xml_node enableSecondaryCamera = root.child("EnableSecondaryCameraServer");
	if(enableSecondaryCamera){
		xml_attribute cameraServerEnabled = enableSecondaryCamera.attribute("value");
		if(cameraServerEnabled.as_bool()){
			cs::UsbCamera cam = CameraServer::GetInstance()->StartAutomaticCapture();
			if(enableSecondaryCamera.attribute("autoExposure").as_bool()){
				cam.SetExposureAuto();
				Log::General("SecondaryCameraServer AutoExposure enabled");
			}
			if(enableSecondaryCamera.attribute("width") && enableSecondaryCamera.attribute("height")){
				cam.SetResolution(enableSecondaryCamera.attribute("width").as_int(), enableSecondaryCamera.attribute("height").as_int());
			}
			else{
				//TODO: Make stuff like this log-only (ie. not console) unless verbose output. Will assign to Ian.
				Log::Error("No width/height for SecondaryCameraServer found!");
			}
			if(enableSecondaryCamera.attribute("fps"))
				cam.SetFPS(enableSecondaryCamera.attribute("fps").as_int());
			else
				Log::Error("No FPS for SecondaryCameraServer found!");
			Log::General(R"(Secondary camera server started, you can access the stream at http://10.34.81.2:1181)", true);
		}
		else{
			Log::Error("EnableSecondaryCameraServer value not found. Disabling by default");
		}
	}
	else
		Log::Error("EnableSecondaryCameraServer not found. Disabling by default");

	#pragma endregion SecondaryCameraServer

	#pragma endregion MetaData

	AllocateComponents(root);

	xml_node controls = root.child("Controls");

	if(controls){
		Log::General("Controls tag found");
	}
	else{
		Log::Error("Control definitions were not found in Config! Returning to Robot.cpp");
		return;
	}

	AllocateDriverControls(controls);
	AllocateOperatorControls(controls);
}

void Config::AllocateComponents(xml_node &root){
	xml_node robot = root.child("RobotConfig");
	if(!robot){
		Log::General("RobotConfig was not found in Config! I hope you didn't intend to allocate any components! Returning to Config::LoadValues()");
		return;
	}

	//TODO: When/if we create a drivetrain class, we will need drive/aux motors
	//TODO: Look into setting encoders to motors like we used to do in the C# code
	//TODO: Upper and lower limits that can be either AI or DI

	#pragma region VictorSP
	
	xml_node VictorSP = robot.child("VictorSP");
	if(VictorSP){
		for(xml_node victorSp = VictorSP.first_child(); victorSp; victorSp = victorSp.next_sibling()){
			string name = victorSp.name();
			xml_attribute channel = victorSp.attribute("channel");
			bool reversed = victorSp.attribute("reversed").as_bool();
			int pdbChannel = victorSp.attribute("pdbChannel") ? victorSp.attribute("pdbChannel").as_int() : -1;
			if(channel){
				VictorSPItem *tmp = new VictorSPItem(name, channel.as_int(), reversed);
				m_activeCollection->Add(tmp);
				string channel_print = channel.as_string();
				string reversed_print = reversed ? "true" : "false" ;
				Log::General("Added VictorSP " + name + ", Channel: " + channel_print + ", Reversed: " + reversed_print);
				if(pdbChannel != -1){
					string pdbChannel_print = to_string(pdbChannel);
					Log::General("Allocated PDBChannel " + pdbChannel_print + " for VictorSP " + name);
					tmp->SetPDBChannel(pdbChannel);
				}
			}
			else{
				Log::Error("Failed to load VictorSP " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("VictorSP definitions not found in config, skipping...");
	}

	#pragma endregion VictorSP

	#pragma region VictorSPX

	xml_node VictorSPX = robot.child("VictorSPX");
	if(VictorSPX){
		for(xml_node victorSpx = VictorSPX.first_child(); victorSpx; victorSpx = victorSpx.next_sibling()){
			string name = victorSpx.name();
			xml_attribute channel = victorSpx.attribute("channel");
			bool reversed = victorSpx.attribute("reversed");
			int pdbChannel = victorSpx.attribute("pdbChannel") ? victorSpx.attribute("pdbChannel").as_int() : -1;
			if(channel){
				VictorSPXItem *tmp = new VictorSPXItem(channel.as_int(), name, reversed);
				m_activeCollection->Add(tmp);
				string reversed_print = reversed ? "true" : "false";
				Log::General("Added VictorSPX " + name + ", Channel: " + to_string(channel.as_int()) + ", Reversed: " + reversed_print);
				if(pdbChannel != -1){
					Log::General("Allocated PDBChannel " + to_string(pdbChannel) + " for VictorSPX " + name);
					tmp->SetPDBChannel(pdbChannel);
				}
			}
			else{
				Log::Error("Failed to load VictorSPX " + name + ". This may cause a fatal runtime error!");
			}


		}
	}
	else{
		Log::Error("VictorSPX definitions not found in config, skipping...");
	}

#pragma endregion VictorSPX

	#pragma region TalonSRX

	xml_node TalonSRX = robot.child("TalonSRX");
	if(TalonSRX){
		for(xml_node talonSrx = TalonSRX.first_child(); talonSrx; talonSrx = talonSrx.next_sibling()){
			string name = talonSrx.name();
			xml_attribute channel = talonSrx.attribute("channel");
			bool reversed = talonSrx.attribute("reversed");
			bool enableEncoder = talonSrx.attribute("enableEncoder");
			int pdbChannel = talonSrx.attribute("pdbChannel") ? talonSrx.attribute("pdbChannel").as_int() : -1;
			if(channel){
				TalonSRXItem *tmp = new TalonSRXItem(channel.as_int(), name, reversed, enableEncoder);
				m_activeCollection->Add(tmp);
				string reversed_print = reversed ? "true" : "false" ;
				string enableEncoder_print = enableEncoder ? "true" : "false" ;
				Log::General("Added TalonSRX " + name + ", Channel: " + to_string(channel.as_int()) + ", Reversed: " + reversed_print + ", EnableEncoder: " + enableEncoder_print);
				if(pdbChannel != -1){
					Log::General("Allocated PDBChannel " + to_string(pdbChannel) + " for TalonSRX " + name);
					tmp->SetPDBChannel(pdbChannel);
				}
			}
			else{
				Log::Error("Failed to load TalonSRX " + name + ". This may cause a fatal runtime error!");
			}


		}
	}
	else{
		Log::Error("TalonSRX definitions not found in config, skipping...");
	}

#pragma endregion TalonSRX

	#pragma region Potentiometer

	xml_node Pot = robot.child("Potentiometer");
	if(Pot){
		for(xml_node pot = Pot.first_child(); pot; pot = pot.next_sibling()){
			string name = pot.name();
			xml_attribute channel = pot.attribute("channel");
			if(channel){
				PotentiometerItem *tmp = new PotentiometerItem(channel.as_int(), name);
				m_activeCollection->Add(tmp);
				Log::General("Added Potentiometer " + name + ", Channel: " + to_string(channel.as_int()));
			}
			else{
				Log::Error("Failed to load Potentiometer " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("Potentiometer definitions not found in config, skipping...");
	}

#pragma endregion Potentiometer

	#pragma region Encoder

	xml_node Encoder = robot.child("Encoder");
	if(Encoder){
		for(xml_node encoder = Encoder.first_child(); encoder; encoder = encoder.next_sibling()){
			string name = encoder.name();
			xml_attribute aChannel = encoder.attribute("aChannel");
			xml_attribute bChannel = encoder.attribute("bChannel");
			bool reversed = encoder.attribute("reversed").as_bool();
			if(aChannel && bChannel){
				EncoderItem *tmp = new EncoderItem(name, aChannel.as_int(), bChannel.as_int(), reversed);
				m_activeCollection->Add(tmp);
				string reversed_print = reversed ? "true" : "false" ;
				Log::General("Added Encoder " + name + ", A-Channel: " + to_string(aChannel.as_int()) + ", B-Channel: " + to_string(bChannel.as_int()) + ", Reversed: " + reversed_print);
			}
			else{
				Log::General("Failed to load Encoder " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("Encoder definitions not found in config, skipping...");
	}

#pragma endregion Encoder

	#pragma region DoubleSolenoid

	xml_node Solenoid = robot.child("DoubleSolenoid");
	if(Solenoid){
		for(xml_node solenoid = Solenoid.first_child(); solenoid; solenoid = solenoid.next_sibling()){
			string name = solenoid.name();
			xml_attribute fChannel = solenoid.attribute("fChannel");
			xml_attribute rChannel = solenoid.attribute("rChannel");
			bool reversed = solenoid.attribute("reversed").as_bool();
			xml_attribute _default = solenoid.attribute("default");
			DoubleSolenoid::Value _def;
			if(_default){
				if(strcmp(_default.as_string(),"reverse") == 0)
					_def = DoubleSolenoid::Value::kReverse;
				else if(strcmp(_default.as_string(),"forward") == 0)
					_def = DoubleSolenoid::Value::kForward;
				else{
					_def = DoubleSolenoid::Value::kOff;
				}
			}else{
				_def = DoubleSolenoid::Value::kOff;
			}
			string reversed_print = reversed ? "true" : "false";
			if(fChannel && rChannel){
				DoubleSolenoidItem *tmp = new DoubleSolenoidItem(name , fChannel.as_int(), rChannel.as_int(), _def, reversed);
				m_activeCollection->Add(tmp);
				Log::General("Added DoubleSolenoid " + name + ", F-Channel: " + to_string(fChannel.as_int()) + ", R-Channel: " + to_string(rChannel.as_int()) + ", Default: " + to_string(_def) + ", Reversed: " + reversed_print);
			}
			else{
				Log::Error("Failed to load DoubleSolenoid " + name + ". This may cause a fatal runtime error!");
			}


		}
	}
	else{
		Log::Error("DoubleSolenoid definitions not found in config, skipping...");
	}

#pragma endregion DoubleSolenoid

	#pragma region DigitalInput

	xml_node DI = robot.child("DI");
	if(DI){
		for(xml_node di = DI.first_child(); di; di = di.next_sibling()){
			string name = di.name();
			xml_attribute channel = di.attribute("channel");
			if(channel){
				DigitalInputItem *tmp = new DigitalInputItem(channel.as_int(), name);
				m_activeCollection->Add(tmp);
				Log::General("Added DigitalInput " + name + ", Channel: " + to_string(channel.as_int()));
			}
			else{
				Log::Error("Failed to load DigitalInput " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("DigitalInput definitions not found in config, skipping...");
	}

#pragma endregion DigitalInput

}

void Config::AllocateDriverControls(xml_node &controls){
	xml_node drive = controls.child("Driver");
	if(!drive){
		Log::Error("Drive Control definitions not found in config! Skipping...");
		return;
	}
	int slot = drive.attribute("slot").as_int();
	Log::General("Configured Driver Joystick at slot " + slot, true);
	m_driveJoy = new Joystick(slot);

	#pragma region AxisControl
	
	xml_node AxisControls = drive.child("AxisControls");
	if(AxisControls){
		for(xml_node axis = AxisControls.first_child(); axis; axis = axis.next_sibling()){
			string name = axis.name();
			xml_attribute channel = axis.attribute("axis");
			if(channel){
				bool reversed = axis.attribute("reversed").as_bool();
				double deadZone;
				double multiply;
				xml_attribute deadZone_xml = axis.attribute("deadZone");
				xml_attribute multiply_xml = axis.attribute("powerMultiplier");
				bool isLift = axis.attribute("isLift").as_bool();
				if(!deadZone_xml){
					Log::Error("No DeadZone detected for AxisControl " + name + ". Defaulting to 0.085. This may cause driving errors!");
					deadZone = 0.085;
				}
				else 
					deadZone = deadZone_xml.as_double();
				if(!multiply_xml){
					Log::Error("No Power Multiplier detected for AxisControl " + name + ". Defaulting to 1.0. This may cause driving errors!");
					multiply = 1.0;
				}
				else
					multiply = multiply_xml.as_double();
				AxisControl *tmp = new AxisControl(m_driveJoy, name, channel.as_int(), deadZone, reversed, multiply, m_activeCollection);
				m_drive->AddControlDrive(tmp);
				string reversed_print = reversed ? "true" : "false" ;
				Log::General("Added AxisControl " + name + ", Axis: " + to_string(channel.as_int()) + ", DeadZone: " + to_string(deadZone) + ", Reversed: " + reversed_print + ", Power Multiplier: " + to_string(multiply) + "Is Lift: " + to_string(isLift));
				xml_attribute bindings = axis.attribute("bindings");
				if(bindings){
					string bind_string = bindings.as_string();
					vector<string> bind_vector = getBindingStringList(bind_string);
					setBindingsToControl(bind_vector, tmp);
				}
				else{
					Log::Error("Control bindings not found for " + name + ". Did you intend to bind this control to anything?");
				}
				if(isLift)
					tmp->SetLift(1.5, m_activeCollection);
				xml_attribute bind_event_xml = axis.attribute("bindEvent");
				bool bind_event = bind_event_xml.as_bool(); 
				if(!bind_event_xml || bind_event){
					m_activeCollection->AddEvent(&(tmp->ValueChanged));
				}
			}
			else{
				Log::Error("Failed to load AxisControl " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("Axis Control Driver definitions not found! Skipping...");
	}
		

	#pragma endregion AxisControl

	#pragma region ButtonControl

	xml_node ButtonControls = drive.child("ButtonControls");
	if(ButtonControls){
		for(xml_node button = ButtonControls.first_child(); button; button = button.next_sibling()){
			string name = button.name();
			xml_attribute channel = button.attribute("button");
			if(channel){
				bool reversed = button.attribute("reversed").as_bool();

				double multiply;
				xml_attribute multiply_xml = button.attribute("powerMultiplier");
				bool actOnRelease = button.attribute("actOnRelease").as_bool();
				bool isSolenoid = button.attribute("isSolenoid").as_bool();
				bool isAmpLimited = button.attribute("isAmpLimited").as_bool();
				bool isRamp = button.attribute("isRamp").as_bool();
				if(!multiply_xml){
					Log::Error("No Power Multiplier detected for ButtonControl " + name + ". Defaulting to 1.0. This may cause driving errors!");
					multiply = 1.0;
				}
				else
					multiply = multiply_xml.as_double();
				ButtonControl *tmp = new ButtonControl(m_driveJoy, name, channel.as_int(), actOnRelease, reversed, multiply, isSolenoid, m_activeCollection);
				m_drive->AddControlDrive(tmp);
				string actOnRelease_print = actOnRelease ? "true" : "false";
				string reversed_print = reversed ? "true" : "false";
				string isSolenoid_print = isSolenoid ? "true" : "false";
				Log::General("Added Button Control " + name + ", Button: " + to_string(channel.as_int()) + ", ActOnRelease: " + actOnRelease_print + ", Reversed: " + reversed_print + ", PowerMultiplier: " + to_string(multiply) + ", IsSolenoid: " + isSolenoid_print);
				xml_attribute bindings = button.attribute("bindings");
				if(bindings){
					string bind_string = bindings.as_string();
					vector<string> bind_vector = getBindingStringList(bind_string);
					setBindingsToControl(bind_vector, tmp);
				}
				else{
					Log::Error("Control bindings not found for " + name + ". Did you intend to bind this control to anything?");
				}
				if(isAmpLimited)
					tmp->SetAmpRegulation(11, 30);
				if (isRamp)
					tmp->SetRamp(0.1);
				//TODO: make this work lol
				xml_attribute bind_event_xml = button.attribute("bindEvent");
				bool bind_event = bind_event_xml.as_bool(); 
				if(!bind_event_xml || bind_event){
					m_activeCollection->AddEvent(&(tmp->ValueChanged));
				}
			}
			else{
				Log::Error("Failed to load ButtonControl " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("Button Control Driver definitions not found! Skipping...");
	}

	#pragma endregion ButtonControl 

	#pragma region ToggleButtonControl

	xml_node ToggleButtonControls = drive.child("ToggleButtonControls");
	if(ToggleButtonControls){
		for(xml_node button = ToggleButtonControls.first_child(); button; button = button.next_sibling()){
			string name = button.name();
			xml_attribute channel = button.attribute("button");
			if(channel){
				bool reversed = button.attribute("reversed").as_bool();
				double multiply;
				xml_attribute multiply_xml = button.attribute("powerMultiplier");
				if(!multiply_xml){
					Log::Error("No Power Multiplier detected for ToggleButtonControl " + name + ". Defaulting to 1.0. This may cause driving errors!");
					multiply = 1.0;
				}
				else
					multiply = multiply_xml.as_double();
				ToggleButtonControl *tmp = new ToggleButtonControl(m_driveJoy, name, channel.as_int(), reversed, multiply, m_activeCollection);
				m_drive->AddControlDrive(tmp);
				xml_attribute bindings = button.attribute("bindings");
				if(bindings){
					string bind_string = bindings.as_string();
					vector<string> bind_vector = getBindingStringList(bind_string);
					setBindingsToControl(bind_vector, tmp);
				}
				else{
					Log::Error("Control bindings not found for " + name + ". Did you intend to bind this control to anything?");
				}
				xml_attribute bind_event_xml = button.attribute("bindEvent");
				bool bind_event = bind_event_xml.as_bool(); 
				if(!bind_event_xml || bind_event){
					m_activeCollection->AddEvent(&(tmp->ValueChanged));
				}
			}
			else{
				Log::Error("Failed to load ToggleButtonControl " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("Toggle Button Control Driver definitions not found! Skipping...");
	}	

	#pragma endregion ToggleButtonControl

	#pragma region GoalButtonControl
	
	xml_node GoalButtonControls = drive.child("GoalButtonControls");
	if(GoalButtonControls){
		for(xml_node button = GoalButtonControls.first_child(); button; button = button.next_sibling()){
			string name = button.name();
			xml_attribute channel = button.attribute("button");
			if(channel){
				xml_attribute goal = button.attribute("goal");
				xml_attribute params = button.attribute("params");
				if(goal && params){
					TeleOpGoal goalToAdd = getTeleOpGoal(goal.as_string());
					if(goalToAdd != TeleOpGoal::None){
						GoalButtonControl* tmp = new GoalButtonControl(m_driveJoy, name, channel.as_int(), m_activeCollection, goalToAdd, params.as_double());
						m_drive->AddControlDrive(tmp);
						Log::General("Added GoalButtonControl " + name + ", Button: " + to_string(channel.as_int()) + ", Goal: " + goal.as_string() + ", Params: " + params.as_string());
						xml_attribute bind_event_xml = button.attribute("bindEvent");
						bool bind_event = bind_event_xml.as_bool(); 
						if(!bind_event_xml || bind_event){
							m_activeCollection->AddEvent(&(tmp->ValueChanged));
						}
					}
					else {
						Log::Error("Failed to load GoalButtonControl " + name + ". This may cause a fatal runtime eror!");
					}
				}
				else{
					Log::Error("Failed to load GoalButtonControl " + name + ". This may cause a fatal runtime eror!");
				}
			}
			else{
				Log::Error("Failed to load GoalButtonControl " + name + ". This may cause a fatal runtime eror!");
			}
		}
	}
	else{
		Log::Error("Goal Button Control Driver definitions not found! Skipping...");
	}

	#pragma endregion
}

void Config::AllocateOperatorControls(xml_node &controls){
	xml_node _operator = controls.child("Operator");
	if(!_operator){
		Log::Error("Operator Control definitions not found in config! Skipping...");
		return;
	}
	int slot = _operator.attribute("slot").as_int();
	Log::General("Configured Operator Joystick at slot " + to_string(slot), true);
	m_operatorJoy = new Joystick(slot);

	#pragma region AxisControl
	
	xml_node AxisControls = _operator.child("AxisControls");
	if(AxisControls){
		for(xml_node axis = AxisControls.first_child(); axis; axis = axis.next_sibling()){
			string name = axis.name();
			xml_attribute channel = axis.attribute("axis");
			if(channel){
				bool reversed = axis.attribute("reversed").as_bool();
				double deadZone;
				double multiply;
				xml_attribute deadZone_xml = axis.attribute("deadZone");
				xml_attribute multiply_xml = axis.attribute("powerMultiplier");
				bool isLift = axis.attribute("isLift").as_bool();
				if(!deadZone_xml){
					Log::Error("No DeadZone detected for AxisControl " + name + ". Defaulting to 0.085. This may cause driving errors!");
					deadZone = 0.085;
				}
				else 
					deadZone = deadZone_xml.as_double();
				if(!multiply_xml){
					Log::Error("No Power Multiplier detected for AxisControl " + name + ". Defaulting to 1.0. This may cause driving errors!");
					multiply = 1.0;
				}
				else
					multiply = multiply_xml.as_double();
				AxisControl *tmp = new AxisControl(m_operatorJoy, name, channel.as_int(), deadZone, reversed, multiply, m_activeCollection);
				m_drive->AddControlOperate(tmp);
				xml_attribute bindings = axis.attribute("bindings");
				if(bindings){
					string bind_string = bindings.as_string();
					vector<string> bind_vector = getBindingStringList(bind_string);
					setBindingsToControl(bind_vector, tmp);
				}
				else{
					Log::Error("Control bindings not found for " + name + ". Did you intend to bind this control to anything?");
				}
				xml_attribute bind_event_xml = axis.attribute("bindEvent");
				bool bind_event = bind_event_xml.as_bool(); 
				if(!bind_event_xml || bind_event){
					m_activeCollection->AddEvent(&(tmp->ValueChanged));
				}
				if(isLift){
					tmp->SetLift(1.5, m_activeCollection);
					Log::General("SET LIFT");
				}
			}
			else{
				Log::Error("Failed to load AxisControl " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("Axis Control Operator definitions not found! Skipping...");
	}
		

	#pragma endregion AxisControl

	#pragma region ButtonControl

	xml_node ButtonControls = _operator.child("ButtonControls");
	if(ButtonControls){
		for(xml_node button = ButtonControls.first_child(); button; button = button.next_sibling()){
			string name = button.name();
			xml_attribute channel = button.attribute("button");
			if(channel){
				bool reversed = button.attribute("reversed").as_bool();
				double multiply;
				xml_attribute multiply_xml = button.attribute("powerMultiplier");
				bool actOnRelease = button.attribute("actOnRelease").as_bool();
				bool isSolenoid = button.attribute("isSolenoid").as_bool();
				bool isAmpLimited = button.attribute("isAmpLimited").as_bool();
				bool isRamp = button.attribute("isRamp").as_bool();
				if(!multiply_xml){
					Log::Error("No Power Multiplier detected for ButtonControl " + name + ". Defaulting to 1.0. This may cause driving errors!");
					multiply = 1.0;
				}
				else
					multiply = multiply_xml.as_double();
				ButtonControl *tmp = new ButtonControl(m_operatorJoy, name, channel.as_int(), actOnRelease, reversed, multiply, isSolenoid, m_activeCollection);
				m_drive->AddControlOperate(tmp);
				xml_attribute bindings = button.attribute("bindings");
				if(bindings){
					string bind_string = bindings.as_string();
					vector<string> bind_vector = getBindingStringList(bind_string);
					setBindingsToControl(bind_vector, tmp);
				}
				else{
					Log::Error("Control bindings not found for " + name + ". Did you intend to bind this control to anything?");
				}
				if(isAmpLimited)
					tmp->SetAmpRegulation(11, 30);
				if(isRamp)
					tmp->SetRamp(0.1);
				xml_attribute bind_event_xml = button.attribute("bindEvent");
				bool bind_event = bind_event_xml.as_bool(); 
				if(!bind_event_xml || bind_event){
					m_activeCollection->AddEvent(&(tmp->ValueChanged));
				}
			}
			else{
				Log::Error("Failed to load ButtonControl " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("Button Control Operator definitions not found! Skipping...");
	}

	#pragma endregion ButtonControl 

	#pragma region ToggleButtonControl

	xml_node ToggleButtonControls = _operator.child("ToggleButtonControls");
	if(ToggleButtonControls){
		for(xml_node button = ToggleButtonControls.first_child(); button; button = button.next_sibling()){
			string name = button.name();
			xml_attribute channel = button.attribute("button");
			if(channel){
				bool reversed = button.attribute("reversed").as_bool();
				double multiply;
				xml_attribute multiply_xml = button.attribute("powerMultiplier");
				if(!multiply_xml){
					Log::Error("No Power Multiplier detected for ToggleButtonControl " + name + ". Defaulting to 1.0. This may cause driving errors!");
					multiply = 1.0;
				}
				else
					multiply = multiply_xml.as_double();
				ToggleButtonControl *tmp = new ToggleButtonControl(m_operatorJoy, name, channel.as_int(), reversed, multiply, m_activeCollection);
				m_drive->AddControlOperate(tmp);
				xml_attribute bindings = button.attribute("bindings");
				if(bindings){
					string bind_string = bindings.as_string();
					vector<string> bind_vector = getBindingStringList(bind_string);
					setBindingsToControl(bind_vector, tmp);
				}
				else{
					Log::Error("Control bindings not found for " + name + ". Did you intend to bind this control to anything?");
				}
				xml_attribute bind_event_xml = button.attribute("bindEvent");
				bool bind_event = bind_event_xml.as_bool(); 
				if(!bind_event_xml || bind_event){
					m_activeCollection->AddEvent(&(tmp->ValueChanged));
				}
			}
			else{
				Log::Error("Failed to load ToggleButtonControl " + name + ". This may cause a fatal runtime error!");
			}
		}
	}
	else{
		Log::Error("Toggle Button Control Driver definitions not found! Skipping...");
	}	

	#pragma endregion ToggleButtonControl

	#pragma region GoalButtonControl
	
	xml_node GoalButtonControls = _operator.child("GoalButtonControls");
	if(GoalButtonControls){
		for(xml_node button = GoalButtonControls.first_child(); button; button = button.next_sibling()){
			string name = button.name();
			xml_attribute channel = button.attribute("button");
			if(channel){
				xml_attribute goal = button.attribute("goal");
				xml_attribute params = button.attribute("params");
				if(goal && params){
					TeleOpGoal goalToAdd = getTeleOpGoal(goal.as_string());
					if(goalToAdd != TeleOpGoal::None){
						GoalButtonControl* tmp = new GoalButtonControl(m_operatorJoy, name, channel.as_int(), m_activeCollection, goalToAdd, params.as_double());
						m_drive->AddControlOperate(tmp);
						Log::General("Added GoalButtonControl " + name + ", Button: " + to_string(channel.as_int()) + ", Goal: " + goal.as_string() + ", Params: " + params.as_string());
						xml_attribute bind_event_xml = button.attribute("bindEvent");
						bool bind_event = bind_event_xml.as_bool(); 
						if(!bind_event_xml || bind_event){
							m_activeCollection->AddEvent(&(tmp->ValueChanged));
						}
					}
					else {
						Log::Error("Failed to load GoalButtonControl " + name + ". This may cause a fatal runtime eror!");
					}
				}
				else{
					Log::Error("Failed to load GoalButtonControl " + name + ". This may cause a fatal runtime eror!");
				}
			}
			else{
				Log::Error("Failed to load GoalButtonControl " + name + ". This may cause a fatal runtime eror!");
			}
		}
	}
	else{
		Log::Error("Goal Button Control Operator definitions not found! Skipping...");
	}

	#pragma endregion

}
vector<string> Config::getBindingStringList(string bindings){
	vector<char*> tmp;
	vector<string> ret;
	//char * pch;   //unreferenced
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

bool Config::setBindingsToControl(vector<string> bindings, ControlItem *control){
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

TeleOpGoal Config::getTeleOpGoal(string goalString){
	if(goalString.compare("ElevatorControl") == 0){
		return TeleOpGoal::ElevatorControl;
	}
	else if (goalString.compare("Timer") == 0) {
		return TeleOpGoal::Timer;
	}
	else if(goalString.compare("DriveWithTimer") == 0)
	{
		return TeleOpGoal::eDriveWithTimer;
	}
	else{
		Log::Error("Error registering teleop goal " + goalString + ". Skipping this control...");
		return TeleOpGoal::None;
	}
}

Config::~Config(){}