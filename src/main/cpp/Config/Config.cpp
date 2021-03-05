/****************************** Header ******************************\
Class Name: Config
File Name:	Config.h
Summary: 	Manages and loads the configuration file.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ryan Cooper, Dylan Watson, Chris Weeks
Email: cooper.ryan@centaurisoftware.co, dylantrwatson@gmail.com, chrisrweeks@aol.com
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

	_activeCollection->DeleteAll();
	_drive->DeleteAll();

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
		try
		{
			Log::General("xml not found...loading backup config");
			backupConfig *_Backup = new backupConfig(_activeCollection, _drive);
		}
		catch(...)
		{
			Log::Error("XML Config and backup config parsed with errors");
			string desc = result.description();
			Log::Error("Error description: " + desc);
			Log::Error("Error offset: " + result.offset);
			Log::Error("No config available, returning to Robot.cpp\nTHIS IS A BIG ERROR!");
			//In simulation we should really get the message across
			#ifdef _Win32
			assert(false);  
			#endif
		}
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

	#pragma region Comp

	bool comp = root.child("Competition").attribute("AtComp").as_bool();
	if(comp)
	{
		Log::atComp = comp;
	}
	else
	{
		Log::atComp = false;
	}

	#pragma endregion Comp

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

	#pragma region TimeLoop

	xml_attribute Loop = root.child("Time").attribute("seconds");
	if(Loop){
		double timeL = Loop.as_double();
		m_activeCollection->SetLoopTime(timeL);
		Log::General("Loop Time set: " + to_string(timeL), true);
	}
	else
		Log::Error("Comment not found");

	#pragma endregion TimeLoop

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

	#pragma region PDB

	if (root.child("PDBManager"))
	{
		xml_attribute MaxCur = root.child("PDBManager").attribute("MaxCurrrent");
		xml_attribute TimeOut = root.child("PDBManager").attribute("TimeOut");
		xml_attribute Lower = root.child("PDBManager").attribute("LowerAmountBy");
		xml_attribute Run = root.child("PDBManager").attribute("Update");
		if (MaxCur && TimeOut)
		{
			m_activeCollection->SetPDP(TimeOut.as_double(), MaxCur.as_double(), Lower.as_double(), Run.as_bool());
		}
	}
	#pragma endregion PDB

	#pragma region AutoSelection

	xml_node AtoE = root.child("Selected_Auto");
	if(AtoE)
	{
		xml_attribute Ato = AtoE.attribute("OverrideDS");
		if(Ato)
		{
			string AtoFile = Ato.as_string();
			Log::General("Selected Auto: " + AtoFile);
			m_activeCollection->SetAuto(AtoFile);
		}

		xml_attribute AtoOverride = AtoE.attribute("OverrideDS");
		if(AtoOverride)
		{
			bool AtoOver = AtoOverride.as_bool();
			m_activeCollection->SetAutoOverride(AtoOver);
		}
		xml_attribute AtoScale = AtoE.attribute("Scale");
		if(AtoScale)
		{
			double Scale = AtoOverride.as_double();
			m_activeCollection->SetAutoScale(Scale);
		}
		xml_attribute Drive = AtoE.attribute("Swerve");
		if(Drive)
		{
			bool TypeDrive = Drive.as_bool();
			m_activeCollection->SetIsSwerveDrive(TypeDrive);
		}
	}
	else
	{
		Log::Error("Auto not found");
	}

	#pragma endregion AutoSelection

	#pragma region ColorSen

	xml_node CS = root.child("ColorSensor");
	if(CS){
		REVColorSensorV3 *tmp;
		if(CS.attribute("name"))
			tmp = new REVColorSensorV3(CS.attribute("name").as_string());
		else
			tmp = new REVColorSensorV3("Color");
		m_activeCollection->Add(tmp);
		Log::General("Added Color Sensor");
	}
	else{
		Log::Error("Color Sensor definitions not found in config, skipping...");
	}

	#pragma endregion ColorSen

	#pragma region SecondaryCameraServer

	//TODO: make it so we can mess with the camera during the running of the robot: ie, switch which stream we are using 
	xml_node enableSecondaryCamera = root.child("RobotCameraServer");
	if(enableSecondaryCamera)
	{
		//TODO: chris dum
		//CameraServer::GetInstance()->StartAutomaticCapture(0);
		if(enableSecondaryCamera.attribute("enabled").as_bool())
		{
			
			for(xml_node camera = enableSecondaryCamera.first_child(); camera; camera = camera.next_sibling())
			{
				Log::General("first for loop");
				if(camera)
				{
				xml_attribute enabled = camera.attribute("enabled");
				if(enabled.as_bool())
				{
					xml_attribute port = camera.attribute("port");
					xml_attribute fps = camera.attribute("fps");
					xml_attribute width = camera.attribute("width");
					xml_attribute height = camera.attribute("height");
					int iport, ifps, iwidth, iheight;
					if(port)
					{
						iport = port.as_int();
					}
					else
					{
						Log::Error("CAMERA IS MISSING PORT! DISABLED!");
						continue;
					}

					if(fps)
					{
						ifps = fps.as_int();
					}
					else
					{
						Log::Error("Camera FPS Missing! Using default value!");
						ifps = 15;
					}
					
					if(width && height)
					{
						iwidth = width.as_int();
						iheight = height.as_int();
					}
					else
					{
						Log::Error("Camera Width or Height Missing! Using default values!");
						iwidth = 160;
						iheight = 120;
					}
					Log::General("iport" + iport,true);


					 cs::UsbCamera cam = CameraServer::GetInstance()->StartAutomaticCapture(iport);
					 cam.SetFPS(ifps);
					 cam.SetResolution(iwidth,iheight);

					
					//cam;
				}
				else
				{
					Log::General("Camera Disabled");
				}
				}
			}
		}
	}
		
		

	#pragma endregion SecondaryCameraServer

	#pragma region Vision
	shared_ptr<NetworkTable> vision_table = nt::NetworkTableInstance::GetDefault().GetTable("VISION_2019");
	xml_node vision = root.child("Vision");
	if(vision)
	{
		//LS
		if(vision.attribute("LS"))
		{
			vision_table->PutNumber("LS",vision.attribute("LS").as_int());
		}
		else
		{
			Log::Error("Vision LS not found! Vision server does not have proper values to work with and will likely fail!");
		}
		//US
		if(vision.attribute("US"))
		{
			vision_table->PutNumber("US",vision.attribute("US").as_int());
		}
		else
		{
			Log::Error("Vision US not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//LH
		if(vision.attribute("LH"))
		{
			vision_table->PutNumber("LH",vision.attribute("LH").as_int());
		}
		else
		{
			Log::Error("Vision LH not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//UH
		if(vision.attribute("UH"))
		{
			vision_table->PutNumber("UH",vision.attribute("UH").as_int());
		}
		else
		{
			Log::Error("Vision UH not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//LV
		if(vision.attribute("LV"))
		{
			vision_table->PutNumber("LV",vision.attribute("LV").as_int());
		}
		else
		{
			Log::Error("Vision LV not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//UV
		if(vision.attribute("UV"))
		{
			vision_table->PutNumber("UV",vision.attribute("UV").as_int());
		}
		else
		{
			Log::Error("Vision UV not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//MinA
		if(vision.attribute("MinA"))
		{
			vision_table->PutNumber("MinA",vision.attribute("MinA").as_int());
		}
		else
		{
			Log::Error("Vision MinA not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//MaxA
		if(vision.attribute("MaxA"))
		{
			vision_table->PutNumber("MaxA",vision.attribute("MaxA").as_int());
		}
		else
		{
			Log::Error("Vision MaxA not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//MaxO
		if(vision.attribute("MaxO"))
		{
			vision_table->PutNumber("MaxO",vision.attribute("MaxO").as_int());
		}
		else
		{
			Log::Error("Vision MaxO not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//Lower bound
		if(vision.attribute("LOWER_BOUND"))
		{
			vision_table->PutNumber("LOWER_BOUND",vision.attribute("LOWER_BOUND").as_int());
		}
		else
		{
			Log::Error("Vision Lower bound not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//upper bound
		if(vision.attribute("UPPER_BOUND"))
		{
			vision_table->PutNumber("UPPER_BOUND",vision.attribute("UPPER_BOUND").as_int());
		}
		else
		{
			Log::Error("Vision Upper bound not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//left bound
		if(vision.attribute("LEFT_BOUND"))
		{
			vision_table->PutNumber("LEFT_BOUND",vision.attribute("LEFT_BOUND").as_int());
		}
		else
		{
			Log::Error("Vision left bound not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
		//right bound
		if(vision.attribute("RIGHT_BOUND"))
		{
			vision_table->PutNumber("RIGHT_BOUND",vision.attribute("RIGHT_BOUND").as_int());
		}
		else
		{
			Log::Error("Vision right bound not found! Vision server does not have proper values to work with and will likely fail!");
		}
		
	}
	else
	{
		Log::Error("Vision not found. Vision server does not have proper values to work with and will likely fail!");
	}
	
	#pragma endregion Vision

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

	#pragma region LimeLight

	xml_node LM = root.child("limeLight");
 	if(LM)
	{
		limelight* lime = new limelight();
		m_activeCollection->Add(lime);
		Log::General("Added Limelight");
	}
	else
	{
		Log::Error("No limelight in RobotConfig...we need that");
	}

	#pragma endregion LimeLight
		
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

				int MotorGroup = victorSp.attribute("Group") ? victorSp.attribute("Group").as_int() : -1;
				if (MotorGroup != -1)
				{
					m_activeCollection->GetPDBManager()->SetMotorGroup(tmp, MotorGroup);
				}

				double PersonalLowerRate = victorSp.attribute("LowerRate") ? victorSp.attribute("LowerRate").as_double() : -1;
				if (PersonalLowerRate != -1)
				{
					tmp->SetLowerRate(PersonalLowerRate);
				}

				double PersonalRegenRate = victorSp.attribute("RegenRate") ? victorSp.attribute("RegenRate").as_double() : -1;
				if (PersonalRegenRate != -1)
				{
					tmp->SetRegenRate(PersonalRegenRate);
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
			//TODO: Fix this line after comp and fix robot configs
			bool reversed = victorSpx.attribute("reversed");//.as_bool();
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

				int MotorGroup = victorSpx.attribute("Group") ? victorSpx.attribute("Group").as_int() : -1;
				if (MotorGroup != -1)
				{
					m_activeCollection->GetPDBManager()->SetMotorGroup(tmp, MotorGroup);
				}

				double PersonalLowerRate = victorSpx.attribute("LowerRate") ? victorSpx.attribute("LowerRate").as_double() : -1;
				if (PersonalLowerRate != -1)
				{
					tmp->SetLowerRate(PersonalLowerRate);
				}

				double PersonalRegenRate = victorSpx.attribute("RegenRate") ? victorSpx.attribute("RegenRate").as_double() : -1;
				if (PersonalRegenRate != -1)
				{
					tmp->SetRegenRate(PersonalRegenRate);
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

#pragma endregion SparkMax

	#pragma region SparkMax
	xml_node SparkMax = robot.child("SparkMax");
	if(SparkMax){
		for(xml_node sparkMax = SparkMax.first_child(); sparkMax; sparkMax = sparkMax.next_sibling()){
			
			string name = sparkMax.name();
			xml_attribute channel = sparkMax.attribute("channel");
			//TODO: Fix this line after comp and fix robot configs
			bool reversed = sparkMax.attribute("reversed").as_bool();
			int pdbChannel = sparkMax.attribute("pdbChannel") ? sparkMax.attribute("pdbChannel").as_int() : -1;
			if(channel){
				SparkMaxItem *tmp = new SparkMaxItem(channel.as_int(), name, reversed, true);
				m_activeCollection->Add(tmp);
				string reversed_print = reversed ? "true" : "false";
				Log::General("Added SparkMax " + name + ", Channel: " + to_string(channel.as_int()) + ", Reversed: " + reversed_print);
				if(pdbChannel != -1){
					Log::General("Allocated PDBChannel " + to_string(pdbChannel) + " for SparkMax " + name);
					tmp->SetPDBChannel(pdbChannel);
				}

				int MotorGroup = sparkMax.attribute("Group") ? sparkMax.attribute("Group").as_int() : -1;
				if (MotorGroup != -1)
				{
					m_activeCollection->GetPDBManager()->SetMotorGroup(tmp, MotorGroup);
				}

				double PersonalLowerRate = sparkMax.attribute("LowerRate") ? sparkMax.attribute("LowerRate").as_double() : -1;
				if (PersonalLowerRate != -1)
				{
					tmp->SetLowerRate(PersonalLowerRate);
				}

				double PersonalRegenRate = sparkMax.attribute("RegenRate") ? sparkMax.attribute("RegenRate").as_double() : -1;
				if (PersonalRegenRate != -1)
				{
					tmp->SetRegenRate(PersonalRegenRate);
				}
			}
			else{
				Log::Error("Failed to load SparkMax " + name + ". This may cause a fatal runtime error!");
			}


		}
	}
	else{
		Log::Error("SparkMax definitions not found in config, skipping...");
	}

#pragma endregion SparkMax

	#pragma region TalonSRX

	xml_node TalonSRX = robot.child("TalonSRX");
	if(TalonSRX){
		for(xml_node talonSrx = TalonSRX.first_child(); talonSrx; talonSrx = talonSrx.next_sibling()){
			string name = talonSrx.name();
			xml_attribute channel = talonSrx.attribute("channel");
			bool reversed = talonSrx.attribute("reversed").as_bool();
			bool enableEncoder = talonSrx.attribute("enableEncoder").as_bool();
			int pdbChannel = talonSrx.attribute("pdbChannel") ? talonSrx.attribute("pdbChannel").as_int() : -1;
			if(channel){
				TalonSRXItem *tmp = new TalonSRXItem(channel.as_int(), name, reversed, enableEncoder, true);
				m_activeCollection->Add(tmp);
				string reversed_print = reversed ? "true" : "false" ;
				string enableEncoder_print = enableEncoder ? "true" : "false" ;
				Log::General("Added TalonSRX " + name + ", Channel: " + to_string(channel.as_int()) + ", Reversed: " + reversed_print + ", EnableEncoder: " + enableEncoder_print);
				if(pdbChannel != -1){
					Log::General("Allocated PDBChannel " + to_string(pdbChannel) + " for TalonSRX " + name);
					tmp->SetPDBChannel(pdbChannel);
				}

				int MotorGroup = talonSrx.attribute("Group") ? talonSrx.attribute("Group").as_int() : -1;
				if (MotorGroup != -1)
				{
					m_activeCollection->GetPDBManager()->SetMotorGroup(tmp, MotorGroup);
				}

				double PersonalLowerRate = talonSrx.attribute("LowerRate") ? talonSrx.attribute("LowerRate").as_double() : -1;
				if (PersonalLowerRate != -1)
				{
					tmp->SetLowerRate(PersonalLowerRate);
				}

				double PersonalRegenRate = talonSrx.attribute("RegenRate") ? talonSrx.attribute("RegenRate").as_double() : -1;
				if (PersonalRegenRate != -1)
				{
					tmp->SetRegenRate(PersonalRegenRate);
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
				PotentiometerItem *tmp = new PotentiometerItem(channel.as_int(), name, true);
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
				EncoderItem *tmp = new EncoderItem(name, aChannel.as_int(), bChannel.as_int(), reversed, true);
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
				DoubleSolenoidItem *tmp = new DoubleSolenoidItem(name , fChannel.as_int(), rChannel.as_int(), _def, reversed, true);
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

	xml_node DI = robot.child("DigitalInput");
	if(DI){
		for(xml_node di = DI.first_child(); di; di = di.next_sibling()){
			string name = di.name();
			xml_attribute channel = di.attribute("channel");
			if(channel){
				DigitalInputItem *tmp = new DigitalInputItem(channel.as_int(), name, true);
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

	#pragma region SwerveModule

	xml_node Modules = robot.child("SwerveModules");
	if (Modules)
	{
		for(xml_node module = Modules.first_child(); module; module = module.next_sibling())
		{
			string name = module.name();
			string SwivelName = module.attribute("swivel").as_string();
			string WheelName = module.attribute("wheel").as_string();
			if (m_activeCollection->Get(SwivelName) != nullptr && m_activeCollection->Get(WheelName) != nullptr)
			{
				double Ticksperrev = module.attribute("swivelTicks").as_double();
				double WheelTicksperrev = module.attribute("wheelTicks").as_double();
				xml_attribute Location = module.attribute("location");
				if(Location)
				{
					SwerveModule *tmp = new SwerveModule(name, (Motor*)m_activeCollection->Get(SwivelName), (Motor*)m_activeCollection->Get(WheelName), Ticksperrev, WheelTicksperrev);
					tmp->SetLocation(Config::GetLocation(Location.as_string()));
					m_activeCollection->Add(tmp);
					Log::General("Added Swerve Module :" + name);
				}
				else
				{
					Log::Error("Swerve Module " + name + " cannot find location specified in the config!");
				}
			}
			else
			{
				Log::Error("Swerve Module " + name + " cannot find one or both motors specified in the config!");
			}
		}
	}

	#pragma endregion SwerveModule

	#pragma region SwerveManager

	if (Modules)
	{
		xml_node Man = robot.child("SwerveManager");
		if (Man)
		{
			string name = Man.attribute("name").as_string();
			bool WaitB = Man.attribute("wait").as_bool();
			double MaxPower = Man.attribute("Max").as_double();
			xml_attribute SwerveModules = Man.attribute("modules");
			
			double length = Man.attribute("length").as_double();
			double width = Man.attribute("width").as_double();
			if (SwerveModules)
			{
				istringstream ss(SwerveModules.as_string());
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
					SwerveManager *Manager = new SwerveManager(name, WaitB, Parts, m_activeCollection->GetNavX(), length, width, (Man.attribute("WheelDiameter") ? Man.attribute("WheelDiameter").as_double() : 1), (Man.attribute("Scale") ? Man.attribute("Scale").as_double() : 1));
					Manager->SetMaxPow(MaxPower);
					m_activeCollection->Add(Manager);
					Log::General("Added Swerve Manager with location");
				}
				else
				{
					Log::Error("One or modules not found!");
				}
			}
			else
			{
				Log::Error("Swerve Manager cannot find one or more swerve modules specified in the code!");
			}
		}
	}

	#pragma endregion SwerveManager

	#pragma region _PID

	xml_node Profiles = robot.child("Profiles");
	if(Profiles){
		for(xml_node P = Profiles.first_child(); P; P = P.next_sibling()){
			string name = P.name();
			xml_attribute Pval = P.attribute("P");
			xml_attribute Ival = P.attribute("I");
			xml_attribute Dval = P.attribute("D");
			xml_attribute MaxChange = P.attribute("Change");
			xml_attribute Bias = P.attribute("Bias");
			xml_attribute Min = P.attribute("Min");
			xml_attribute Max = P.attribute("Max");
			xml_attribute Index = P.attribute("Index");
			if(Pval && Ival && Dval)
			{
				if(MaxChange && Bias)
				{
					if(Min)
					{
						if(Max)
						{
							if(Index)
								m_activeCollection->CreateAndAddProfile(name, Pval.as_double(), Ival.as_double(), Dval.as_double(), MaxChange.as_double(), Bias.as_double(), Min.as_double(), Max.as_double(), Index.as_int());
							else
								m_activeCollection->CreateAndAddProfile(name, Pval.as_double(), Ival.as_double(), Dval.as_double(), MaxChange.as_double(), Bias.as_double(), Min.as_double(), Max.as_double());
						}
						else
							m_activeCollection->CreateAndAddProfile(name, Pval.as_double(), Ival.as_double(), Dval.as_double(), MaxChange.as_double(), Bias.as_double(), Min.as_double());
					}
					else
					{
						m_activeCollection->CreateAndAddProfile(name, Pval.as_double(), Ival.as_double(), Dval.as_double(), MaxChange.as_double(), Bias.as_double());
					}
				}
				else
				{
					m_activeCollection->CreateAndAddProfile(name, Pval.as_double(), Ival.as_double(), Dval.as_double());
				}
			}
			else
			{
				Log::Error("Profile not complete");
			}
		}
	}
	else{
		Log::Error("PID Profiles definitions not found in config, skipping...");
	}

	#pragma endregion _PID

	#pragma region Link_PID_Power

	xml_node PowerProfiles = robot.child("PowerLinks");
	if(PowerProfiles){
		for(xml_node P = PowerProfiles.first_child(); P; P = P.next_sibling()){
			string name = P.name();
			xml_attribute MotorName = P.attribute("Motor");
			xml_attribute ProfileName = P.attribute("Profile");
			xml_attribute ProfileIndex = P.attribute("ProfileIndex");
			if(MotorName)
			{
				if(ProfileName)
				{
					string Name = MotorName.as_string();
					Motor* MotorPtr = (Motor*)m_activeCollection->Get(Name);
					if(MotorPtr != nullptr)
					{
						MotorPtr->SetPowerProfile(m_activeCollection->GetProfile(ProfileName.as_string()));
						Log::General("Correctly Set Motor: " + Name + " to PID numbers from: " + Name);
					}
					else
					{
						Log::Error("Motor " + Name + " does not exist!");
					}
				}
				else if(ProfileIndex)
				{
					string Name = MotorName.as_string();
					Motor* MotorPtr = (Motor*)m_activeCollection->Get(Name);
					if(MotorPtr != nullptr)
					{
						MotorPtr->SetPowerProfile(m_activeCollection->GetProfile(ProfileIndex.as_int()));
						Log::General("Correctly Set Motor: " + Name + " to PID numbers from index: " + Name);
					}
					else
					{
						Log::Error("Motor " + Name + " does not exist!");
					}
				}
				else
				{
					Log::Error("Link not complete!");
				}
			}
			else
			{
				Log::Error("Link not complete!");
			}
		}
	}
	else{
		Log::Error("Power Link definitions not found in config, skipping...");
	}

	#pragma endregion Link_PID_Power

	#pragma region Link_PID_Position

	xml_node PositionProfiles = robot.child("PositionLinks");
	if(PositionProfiles){
		for(xml_node P = PositionProfiles.first_child(); P; P = P.next_sibling()){
			string name = P.name();
			xml_attribute MotorName = P.attribute("Motor");
			xml_attribute ProfileName = P.attribute("Profile");
			xml_attribute ProfileIndex = P.attribute("ProfileIndex");
			if(MotorName)
			{
				if(ProfileName)
				{
					string Name = MotorName.as_string();
					Motor* MotorPtr = (Motor*)m_activeCollection->Get(Name);
					if(MotorPtr != nullptr)
					{
						MotorPtr->SetPositonProfile(m_activeCollection->GetProfile(ProfileName.as_string()));
						Log::General("Correctly Set Motor: " + Name + " to PID numbers from: " + Name);
					}
					else
					{
						Log::Error("Motor " + Name + " does not exist!");
					}
				}
				else if(ProfileIndex)
				{
					string Name = MotorName.as_string();
					Motor* MotorPtr = (Motor*)m_activeCollection->Get(Name);
					if(MotorPtr != nullptr)
					{
						MotorPtr->SetPositonProfile(m_activeCollection->GetProfile(ProfileIndex.as_int()));
						Log::General("Correctly Set Motor: " + Name + " to PID numbers from index: " + Name);
					}
					else
					{
						Log::Error("Motor " + Name + " does not exist!");
					}
				}
				else
				{
					Log::Error("Link not complete!");
				}
			}
			else
			{
				Log::Error("Link not complete!");
			}
		}
	}
	else{
		Log::Error("Position Link definitions not found in config, skipping...");
	}

	#pragma endregion Link_PID_Position

	#pragma region Encoder_Links

	xml_node EncoderLinks = robot.child("EncoderLinks");
	if(EncoderLinks){
		for(xml_node P = EncoderLinks.first_child(); P; P = P.next_sibling()){
			string name = P.name();
			xml_attribute MotorName = P.attribute("Motor");
			xml_attribute EncoderName = P.attribute("Encoder");
			if(MotorName)
			{
				if(EncoderName)
				{
					string MotName = MotorName.as_string();
					string EncName = EncoderName.as_string();
					if (m_activeCollection->GetEncoder(EncName) != nullptr)
					{
						if(m_activeCollection->Get(MotName) != nullptr)
						{
							((Motor*)m_activeCollection->Get(MotName))->SetLinkedEncoder(m_activeCollection->GetEncoder(EncName));
						}
						else
						{
							Log::Error(MotName + " doesnt exist!");
						}
					}
					else
					{
						Log::Error(EncName + " doesnt exist!");
					}
				}
				else
				{
					Log::Error("Link not complete!");
				}
			}
			else
			{
				Log::Error("Link not complete!");
			}
		}
	}
	else{
		Log::Error("Encoder Link definitions not found in config, skipping...");
	}

	#pragma endregion Encoder_Links

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
				bool useOverdrive = axis.attribute("useOverdrive").as_bool();
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
				AxisControl *tmp = new AxisControl(m_driveJoy, name, channel.as_int(), deadZone, reversed, multiply, m_activeCollection, useOverdrive);
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
				bool isOverdrive = button.attribute("isOverdrive").as_bool();
				if(!multiply_xml){
					Log::Error("No Power Multiplier detected for ButtonControl " + name + ". Defaulting to 1.0. This may cause driving errors!");
					multiply = 1.0;
				}
				else
					multiply = multiply_xml.as_double();
				ButtonControl *tmp = new ButtonControl(m_driveJoy, name, channel.as_int(), actOnRelease, reversed, multiply, isSolenoid, m_activeCollection, isOverdrive);
				m_drive->AddControlDrive(tmp);
				string actOnRelease_print = actOnRelease ? "true" : "false";
				string reversed_print = reversed ? "true" : "false";
				string isSolenoid_print = isSolenoid ? "true" : "false";
				string isOverdrive_print = isOverdrive ? "true" : "false";
				Log::General("Added Button Control " + name + ", Button: " + to_string(channel.as_int()) + ", ActOnRelease: " + actOnRelease_print + ", Reversed: " + reversed_print + ", PowerMultiplier: " + to_string(multiply) + ", IsSolenoid: " + isSolenoid_print + ", IsOverdrive: " + isOverdrive_print);
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
				xml_attribute ID = button.attribute("ID");
				xml_attribute OtherKeys = button.attribute("RemoveKeys");
				if(goal && params && ID && OtherKeys){
					TeleOpGoal goalToAdd = getTeleOpGoal(goal.as_string());
					if(goalToAdd != TeleOpGoal::eNone){
						
						vector<int> RemKeys;
						vector<string> bind_vector_Key = getBindingStringList(OtherKeys.as_string());
						for(int i = 0; i < bind_vector_Key.size(); i++)
							RemKeys.push_back(stoi(bind_vector_Key.at(i)));

						GoalButtonControl* tmp = new GoalButtonControl(m_driveJoy, name, channel.as_int(), m_activeCollection, goalToAdd, params.as_double(), ID.as_int(), RemKeys);
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

	#pragma region GoalAxisControl
	
	xml_node GoalAxisControls = drive.child("GoalAxisControls");
	if(GoalAxisControls){
		for(xml_node button = GoalAxisControls.first_child(); button; button = button.next_sibling()){
			string name = button.name();
				xml_attribute goal = button.attribute("goal");
				xml_attribute repeat = button.attribute("repeat");
				xml_attribute ID = button.attribute("ID");
				xml_attribute IndexStart = button.attribute("StartIndex");
				xml_attribute OtherKeys = button.attribute("RemoveKeys");
				xml_attribute AxisNumbers = button.attribute("Axis");
				xml_attribute StringsData = button.attribute("StringData");
				xml_attribute DeadZ = button.attribute("DeadZone");
				xml_attribute Mult = button.attribute("mult");
				if(goal && repeat && ID && OtherKeys && AxisNumbers && StringsData && DeadZ && Mult){
					TeleOpGoal goalToAdd = getTeleOpGoal(goal.as_string());
					if(goalToAdd != TeleOpGoal::eNone){

						vector<int> RemKeys;
						vector<string> bind_vector_Key = getBindingStringList(OtherKeys.as_string());
						for(int i = 0; i < bind_vector_Key.size(); i++)
							RemKeys.push_back(stoi(bind_vector_Key.at(i)));

						vector<int> Axis;
						vector<string> bind_vector_Axis = getBindingStringList(AxisNumbers.as_string());
						for(int i = 0; i < bind_vector_Axis.size(); i++)
							RemKeys.push_back(stoi(bind_vector_Axis.at(i)));

						vector<string> StringDataVector = getBindingStringList(StringsData.as_string());

						GoalAxisControl* tmp = new GoalAxisControl(m_driveJoy, name, Axis, m_activeCollection, goalToAdd, StringDataVector, IndexStart.as_int(), ID.as_int(), RemKeys, repeat.as_bool(), DeadZ.as_double(), Mult.as_double());
						m_drive->AddControlOperate(tmp);
						Log::General("Added GoalAxisControl " + name + ", Goal: " + goal.as_string());
						xml_attribute bind_event_xml = button.attribute("bindEvent");
						bool bind_event = bind_event_xml.as_bool(); 
						if(!bind_event_xml || bind_event){
							m_activeCollection->AddEvent(&(tmp->ValueChanged));
						}
					}
					else {
						Log::Error("Failed to load GoalAxisControl " + name + ". This may cause a fatal runtime eror!");
					}
				}
				else{
					Log::Error("Failed to load GoalAxisControl " + name + ". This may cause a fatal runtime eror!");
				}
		}
	}
	else{
		Log::Error("GoalAxisControl Driver definitions not found! Skipping...");
	}

	#pragma endregion

	#pragma region SwerveControl

	xml_node Swerve = drive.child("SwerveControl");
	if (Swerve)
	{
		string drivemode = Swerve.attribute("driveMode").as_string();
		string name = Swerve.attribute("name").as_string();
		int H = Swerve.attribute("H-Axis").as_int();
		int V = Swerve.attribute("V-Axis").as_int();
		int S = Swerve.attribute("S-Axis").as_int();
		double dz = Swerve.attribute("deadZone").as_double();
		double mult = Swerve.attribute("powerMultiplier").as_double();
		bool reversed = Swerve.attribute("reversed").as_bool();
		string ManagerName = Swerve.attribute("manager").as_string();

		SwerveControl::DriveCalculation Cal;
		if (drivemode.compare("Field") == 0)
		{
			Cal = SwerveControl::DriveCalculation::Field_Oriented;
		}
		else if (drivemode.compare("Robot") == 0)
		{
			Cal = SwerveControl::DriveCalculation::Robot_Oriented;
		}
		else if (drivemode.compare("Warthog") == 0)
		{
			Cal = SwerveControl::DriveCalculation::Warthog;
		}
		else if (drivemode.compare("Field_Warthog") == 0)
		{
			Cal = SwerveControl::DriveCalculation::Warthog_Field_Oriented;
		}
		else
		{
			Cal = SwerveControl::DriveCalculation::Robot_Oriented;
		}

		if (m_activeCollection->Get(ManagerName) != nullptr)
		{
			SwerveControl *Control = new SwerveControl(m_driveJoy, Cal, name, V, H, S, dz, reversed, mult, m_activeCollection, (SwerveManager*)m_activeCollection->Get(ManagerName));
			m_drive->AddControlDrive(Control);
			Log::General("Added swerve control that is " + drivemode + " oriented");
		}
		else
		{
			Log::Error("Swerve control cannot find manager with name " + ManagerName);
		}
	}

	#pragma endregion SwerveControl
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
				xml_attribute bind_event_xml = axis.attribute("bindEvent");
				bool bind_event = bind_event_xml.as_bool(); 
				if(!bind_event_xml || bind_event){
					m_activeCollection->AddEvent(&(tmp->ValueChanged));
				}
				if(isLift){
					tmp->SetLift(6.9, m_activeCollection);
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
				xml_attribute ID = button.attribute("ID");
				xml_attribute OtherKeys = button.attribute("RemoveKeys");
				if(goal && params && ID && OtherKeys){
					TeleOpGoal goalToAdd = getTeleOpGoal(goal.as_string());
					if(goalToAdd != TeleOpGoal::eNone){

						vector<int> RemKeys;
						vector<string> bind_vector_Key = getBindingStringList(OtherKeys.as_string());
						for(int i = 0; i < bind_vector_Key.size(); i++)
							RemKeys.push_back(stoi(bind_vector_Key.at(i)));

						GoalButtonControl* tmp = new GoalButtonControl(m_operatorJoy, name, channel.as_int(), m_activeCollection, goalToAdd, params.as_double(), ID.as_int(), RemKeys);
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

	#pragma region GoalAxisControl
	
	xml_node GoalAxisControls = _operator.child("GoalAxisControls");
	if(GoalAxisControls){
		for(xml_node button = GoalAxisControls.first_child(); button; button = button.next_sibling()){
			string name = button.name();
				xml_attribute goal = button.attribute("goal");
				xml_attribute repeat = button.attribute("repeat");
				xml_attribute ID = button.attribute("ID");
				xml_attribute IndexStart = button.attribute("StartIndex");
				xml_attribute OtherKeys = button.attribute("RemoveKeys");
				xml_attribute AxisNumbers = button.attribute("Axis");
				xml_attribute StringsData = button.attribute("StringData");
				xml_attribute DeadZ = button.attribute("DeadZone");
				xml_attribute Mult = button.attribute("mult");
				if(goal && repeat && ID && OtherKeys && AxisNumbers && StringsData && DeadZ && Mult){
					TeleOpGoal goalToAdd = getTeleOpGoal(goal.as_string());
					if(goalToAdd != TeleOpGoal::eNone){

						vector<int> RemKeys;
						vector<string> bind_vector_Key = getBindingStringList(OtherKeys.as_string());
						for(int i = 0; i < bind_vector_Key.size(); i++)
							RemKeys.push_back(stoi(bind_vector_Key.at(i)));

						vector<int> Axis;
						vector<string> bind_vector_Axis = getBindingStringList(AxisNumbers.as_string());
						for(int i = 0; i < bind_vector_Axis.size(); i++)
							RemKeys.push_back(stoi(bind_vector_Axis.at(i)));

						vector<string> StringDataVector = getBindingStringList(StringsData.as_string());

						GoalAxisControl* tmp = new GoalAxisControl(m_operatorJoy, name, Axis, m_activeCollection, goalToAdd, StringDataVector, IndexStart.as_int(), ID.as_int(), RemKeys, repeat.as_bool(), DeadZ.as_double(), Mult.as_double());
						m_drive->AddControlOperate(tmp);
						Log::General("Added GoalAxisControl " + name + ", Goal: " + goal.as_string());
						xml_attribute bind_event_xml = button.attribute("bindEvent");
						bool bind_event = bind_event_xml.as_bool(); 
						if(!bind_event_xml || bind_event){
							m_activeCollection->AddEvent(&(tmp->ValueChanged));
						}
					}
					else {
						Log::Error("Failed to load GoalAxisControl " + name + ". This may cause a fatal runtime eror!");
					}
				}
				else{
					Log::Error("Failed to load GoalAxisControl " + name + ". This may cause a fatal runtime eror!");
				}
		}
	}
	else{
		Log::Error("GoalAxisControl Operator definitions not found! Skipping...");
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
		return TeleOpGoal::eElevatorControl;
	}
	else if (goalString.compare("Timer") == 0) {
		return TeleOpGoal::eTimer;
	}
	else if(goalString.compare("DriveWithTimer") == 0)
	{
		return TeleOpGoal::eDriveWithTimer;
	}
	else if(goalString.compare("RelativeElevatorControl") == 0)
	{
		return TeleOpGoal::eRelativeElevatorControl;
	}
	else if(goalString.compare("ShooterGoal") == 0)
	{
		return TeleOpGoal::eShooter;
	}
	else if(goalString.compare("MotorPosition") == 0)
	{
		return TeleOpGoal::eMotorPosition;
	}
	else{
		Log::Error("Error registering teleop goal " + goalString + ". Skipping this control...");
		return TeleOpGoal::eNone;
	}
}

int Config::GetLocation(string Loc)
{
	if(Loc.compare("Front_Left") == 0){
		return 0;
	}
	else if(Loc.compare("Front_Right") == 0)
	{
		return 1;
	}
	else if(Loc.compare("Back_Left") == 0)
	{
		return 2;
	}
	else if(Loc.compare("Back_Right") == 0)
	{
		return 3;
	}
	else{
		Log::Error("Defaulting location to SwerveModule::Location::Front_Left");
		return 0;
	}
}

Config::~Config(){}