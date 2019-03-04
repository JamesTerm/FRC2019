#ifndef _Win32
#include "frc/WPILib.h"
#endif

#include "../Base/Base_Includes.h"
#include <stdarg.h>
#include <assert.h>

std::string BuildString2(const char *format, va_list argptr)
{
	char Temp[2048];
	vsprintf(Temp,format,argptr);
	std::string ret(Temp);
	return ret;
}


void Dout(size_t line, const char *writeFmt, va_list argptr)
{
	return; //todo
#if 0
	//making this static will avoid needing to define it each time (its a singleton so it is fine)
	static DriverStationLCD *lcd = DriverStationLCD::GetInstance();
	//Since there is no va_list type of method available, we'll build the string ourselved and pass no additional arguments
	//This should avoid problems with the arguments being misaligned.
	if (line!=(size_t)-1)
		lcd->PrintfLine((DriverStationLCD::Line)line, BuildString2(writeFmt,argptr).c_str());
#endif
}

void Dout(size_t line, size_t startingColumn, const char *writeFmt, va_list argptr)
{
	return; //todo
#if 0
	static DriverStationLCD *lcd = DriverStationLCD::GetInstance();
	if (line!=(size_t)-1)
		lcd->Printf((DriverStationLCD::Line)line,startingColumn, BuildString2(writeFmt,argptr).c_str());
#endif
}

void Dout(size_t line, const char *writeFmt, ...)
{
	// Get the arguments
	va_list args;
	va_start( args , writeFmt );
	Dout(line,writeFmt,args);
}

void Dout(size_t line, size_t startingColumn, const char *writeFmt, ...)
{
	// Get the arguments
	va_list args;
	va_start( args , writeFmt );
	Dout(line,startingColumn,writeFmt,args);
}
