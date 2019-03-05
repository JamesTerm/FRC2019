#pragma once
#include "../../RobotLibraries/Common/SmartDashboard.h"

class NetworkTable
{
public:
	void PutBoolean(std::string keyName, bool value);
	bool GetBoolean(std::string keyName, bool defaultValue);

	void PutNumber(std::string keyName, double value);
	double GetNumber(std::string keyName, double defaultValue);

	void PutString(std::string keyName, std::string value);
	//static int GetString(std::string keyName, char *value, unsigned int valueLen);
	//static std::string GetString(std::string keyName);
	std::string GetString(std::string keyName, std::string defaultValue);
	//Not supported in this version of SmartDashboard
	bool PutStringArray(std::string key, std::string value[]) { return false; }
};

namespace nt
{

class NetworkTableInstance
{
public:
	static	NetworkTableInstance &GetDefault();
	std::shared_ptr<NetworkTable> GetTable(const char *keyname);
};


}

