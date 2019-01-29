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

using namespace std;
using namespace System;
using namespace Configuration;
using namespace pugi;
using namespace frc;

/**
 * TODO: all of this stuff lol
 * Load in the document
 * Config Version and Comment (different from the Robot.cpp Version and Comment)
 * MISSING: Verbose Output
 * Probably Post-Season
 * ? AutonEnabled
 * Secondary Camera Server
 * * Vision Initialization -> need vision working with that
 * Allocate Components
 * 	Encoders
 * 	DI
 * 	DO?
 * 	AI?
 * 	AO?
 * 	Motors
 * 		Drive class integration? Probably Post-Season
 * 	Solenoids
 * 	Relays?
 * 	Potentiometers
 * Allocate the Joysticks via the XML
 * Allocate Controls (make another method folsr this)
 * 
**/ 
Config::Config(ActiveCollection *_activeCollection, Drive *_drive) {
//TODO: make doc a member variable?
	xml_document doc;
	xml_parse_result result = doc.load_file("config.xml");
	m_activeCollection = _activeCollection;
	m_drive = _drive;
	if (result)
	{
		cout << "XML Config parsed without errors\n" << endl;
		LoadValues(doc);
	}
	else
	{
		cout << "XML Config parsed with errors" << endl;
		cout << "Error description: " << result.description() << endl;
		cout << "Error offset: " << result.offset << endl;;
		cout << "No config available, returning to Robot.cpp\nTHIS IS A BIG ERROR!" << endl;
	}
}

void Config::LoadValues(xml_document &doc){
	xml_node root = doc.child("Robot");
	
	if(root){
		cout << "Config found" << endl;
	}
	else{
		cout << "Robot was not found in Config! I hope you didn't intend to configure anything! Returning to Robot.cpp" << endl;
		return;
	}

	#pragma region MetaData

	xml_attribute version = root.child("Version").attribute("version");
	if(version)
		cout << "Config Version: " << version.as_int() << endl;
	else
		cout << "Version not found" << endl;

	xml_attribute comment = root.child("Comment").attribute("comment");
	if(comment)
		cout << "Comment: " << comment.as_string() << endl;
	else
		cout << "Comment not found" << endl;

	xml_attribute useNavX = root.child("UseNavX").attribute("value");
	//TODO: Addition of other NavX ports
	if(useNavX){
		if(useNavX.as_bool()){
			NavX *nav = new NavX();
			//m_activeCollection->Add(new NavX());
			cout << "NavX detected" << endl;
		}
		else
			cout << "Failed to load the NavX: Disabling NavX by default" << endl;
			
	}
	else
		cout << "UseNavX not found. Disabling by default" << endl;

	//TODO: make it so we can mess with the camera during the running of the robot: ie, switch which stream we are using 
	xml_node enableSecondaryCamera = root.child("EnableSecondaryCameraServer");
	if(enableSecondaryCamera){
		xml_attribute cameraServerEnabled = enableSecondaryCamera.attribute("value");
		if(cameraServerEnabled.as_bool()){
			cs::UsbCamera cam = CameraServer::GetInstance()->StartAutomaticCapture();
			if(enableSecondaryCamera.attribute("autoExposure").as_bool()){
				cam.SetExposureAuto();
				cout << "SecondaryCameraServer AutoExposure enabled" << endl;
			}
			if(enableSecondaryCamera.attribute("width") && enableSecondaryCamera.attribute("height")){
				cam.SetResolution(enableSecondaryCamera.attribute("width").as_int(), enableSecondaryCamera.attribute("height").as_int());
			}
			else{
				//TODO: Make stuff like this log-only (ie. not console) unless verbose output. Will assign to Ian.
				cout << "No width/height for SecondaryCameraServer found!" << endl;
			}
			if(enableSecondaryCamera.attribute("fps"))
				cam.SetFPS(enableSecondaryCamera.attribute("fps").as_int());
			else
				cout << "No FPS for SecondaryCameraServer found!" << endl;
			cout << R"(Secondary camera server started, you can access the stream at http://10.34.81.2:1181)";
		}
		else{
			cout << "EnableSecondaryCameraServer value not found. Disabling by default" << endl;
		}
	}
	else
		cout << "EnableSecondaryCameraServer not found. Disabling by default" << endl;

	#pragma endregion MetaData
}

void Config::AllocateComponents(){

	VictorSPItem *left_0 = new VictorSPItem("Left_0", 2, true);
	VictorSPItem *left_1 = new VictorSPItem("Left_1", 3, true);
	VictorSPItem *right_0 = new VictorSPItem("Right_0", 0, false);
	VictorSPItem *right_1 = new VictorSPItem("Right_1", 1, false);
	DoubleSolenoidItem *SolenoidTest = new DoubleSolenoidItem("SolenoidToggle", 1, 0, DoubleSolenoid::Value::kReverse, false);

	m_activeCollection->Add(left_0);
	m_activeCollection->Add(left_1);
	m_activeCollection->Add(right_0);
	m_activeCollection->Add(right_1);
	m_activeCollection->Add(SolenoidTest);

	AxisControl *leftDrive = new AxisControl(m_driveJoy, "LeftDrive", 1, 0.07, true, 0.70);
	AxisControl *rightDrive = new AxisControl(m_driveJoy, "RightDrive", 5, 0.07, true, 0.70);
	ToggleButtonControl *ToggleTest = new ToggleButtonControl(m_driveJoy, "ToggleTest", 3, true, false, 0.3, true);

	m_drive->AddControlDrive(leftDrive);
	m_drive->AddControlDrive(rightDrive);
	m_drive->AddControlDrive(ToggleTest);


	leftDrive->AddComponent(left_0);
	leftDrive->AddComponent(left_1);
	rightDrive->AddComponent(right_0);
	rightDrive->AddComponent(right_1);
	ToggleTest->AddComponent(SolenoidTest);
	

}

Config::~Config(){}