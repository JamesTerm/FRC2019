#include "NetworkTable.h"


std::shared_ptr<NetworkTable> g_pNetworkTable;
#define __HideValues__

void NetworkTable::PutBoolean(std::string keyName, bool value)
{
	#ifndef __HideValues__
	SmartDashboard::PutBoolean(keyName, value);
	#endif
}
bool NetworkTable::GetBoolean(std::string keyName, bool defaultValue)
{
	#ifndef __HideValues__
	return SmartDashboard::GetBoolean(keyName, defaultValue);
	#else
	return false;
	#endif
}

void NetworkTable::PutNumber(std::string keyName, double value)
{
	#ifndef __HideValues__
	SmartDashboard::PutNumber(keyName, value);
	#endif
}
double NetworkTable::GetNumber(std::string keyName, double defaultValue)
{
	#ifndef __HideValues__
	return SmartDashboard::GetNumber(keyName, defaultValue);
	#else
	return false;
	#endif
}

void NetworkTable::PutString(std::string keyName, std::string value)
{
	#ifndef __HideValues__
	SmartDashboard::PutString(keyName, value);
	#endif
}
//int NetworkTable::GetString(std::string keyName, char *value, unsigned int valueLen);
//std::string NetworkTable::GetString(std::string keyName);
std::string NetworkTable::GetString(std::string keyName, std::string defaultValue)
{
	#ifndef __HideValues__
	return SmartDashboard::GetString(keyName, defaultValue);
	#else
	return defaultValue;
	#endif
}

namespace nt
{

NetworkTableInstance g_DefaultInstance;

NetworkTableInstance &NetworkTableInstance::GetDefault()
{
	return g_DefaultInstance;
}

std::shared_ptr<NetworkTable> NetworkTableInstance::GetTable(const char *keyname)
{
	if (g_pNetworkTable == nullptr)
		g_pNetworkTable = std::make_shared<NetworkTable>();
	return g_pNetworkTable;
}

}
