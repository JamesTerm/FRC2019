/****************************** Header ******************************\
Class Name: Log
File Name:	Log.h
Summary: Logs
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#pragma once

#ifndef SRC_UTIL_LOG_H_
#define SRC_UTIL_LOG_H_

#include <iostream>
#include <fstream>
#include <frc/WPILib.h>

using namespace std;

namespace Logger
{

	class Log
    	{
        	public:
				static void Error(string message);
				static void General(string message, bool toDriverStation = false);
				static void Warning(string message);
				static void restartfile();
				static void closeLogFile();
				static bool atComp = false;
			private:
				//static const string filename;
				//static ofstream file_;
				static void Append(string message);				
    	};

}

#endif