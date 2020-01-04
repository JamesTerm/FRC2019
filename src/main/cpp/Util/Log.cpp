/****************************** Header ******************************\
Class Name:	Log
File Name:	Log.cpp
Summary: 	Logging
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s):	Ian Poll
Email:	irobot9803@gmail.com
\*********************************************************************/

#include "Log.h"
#include <iostream>
#include <fstream>
#include <cstdio>

using namespace std;
using namespace Logger;
using namespace frc;

    const string filename = "C/RoboLog.txt"; //name of file to be created
    static ofstream file_;
    bool Log::atComp;

    void Log::General(string message, bool toDriverStation)
    {
        if(!atComp)
        {
            if(toDriverStation)
            {
                DriverStation::ReportError("message");
            }
            cout << message << endl;
	    }
        Log::Append("LOG: " + message);

    }
    void Log::Error(string message)
    {
		cout << "ERROR: " + message << endl; //displays Error
        Log::Append("ERROR: " + message);
	}

	void Log::Warning(string message)
    {
		cout << "WARNING: " + message << endl; //displays Warning
		Log::Append("WARNING: " + message);
	}

    void Log::restartfile()
    {
        file_.open(filename); //opens file then clears it
        cout << "File " + filename + " created" << endl;
    }

    void Log::Append(string message)
    {
        if(file_.is_open())
        {
            file_ << message << endl; //appends message in file
            for(int i = 0; i < 1024 - message.length(); i++){
                file_ << " ";
            }
        }
        else
        {
            cout << "File not open" << endl;
        }   
    }

    void Log::closeLogFile()
    {
        if(file_.is_open())
        {
            file_.close();
            cout << "File closed" << endl;
        }
    }
