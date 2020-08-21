/****************************** Header ******************************\
Class Name: -
File Name: LinePaths.h
Summary: File of paths for auto use.
Project: FRC2020
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll (Mostly Zain lets be honest)
Email: irobot9803@gmail.com
\********************************************************************/

#pragma once

#ifndef SRC_UTIL_LINEPATHS_H_
#define SRC_UTIL_LINEPATHS_H_

using namespace std;
using namespace Util;
using namespace Logger;

#define NumPoints 6

struct Point
{
    double Dist;
    double Angle;
    double Act;
    double Speed;
    double TurnAngle;
    double TurnRadius;
};

struct Auto
{
    Auto(int MaxPoints, vector<double> Points)
    {
        for(int i = 0; i < MaxPoints * NumPoints; i+=NumPoints)
        {
            Point* CheckPoint = new Point();
            CheckPoint->Dist = Points[i];
            CheckPoint->Angle = Points[i + 1];
            CheckPoint->Act = Points[i + 2];
            CheckPoint->Speed = Points[i + 3];
            CheckPoint->TurnAngle = Points[i + 4];
            CheckPoint->TurnRadius = Points[i + 5];
            Waypoints.push_back(CheckPoint);
        }
        Num = MaxPoints;
    }
    int Num = 0;
    vector <Point*> Waypoints;
};

static Auto Map(string Path)
{
    ifstream Inputfile (Path);
    string NumberInput;
    if(Inputfile.is_open())
    {
        Log::General("Found File");
        vector<double> InputPoints;
        getline(Inputfile, NumberInput);
        double Length = stod(NumberInput);
        for(int i = 0; i < (Length); i++)
        {
            getline(Inputfile, NumberInput);
            InputPoints.push_back(stod(NumberInput));
            Log::General("Line: " + to_string(i) + " = " + to_string(stod(NumberInput)));
        }
        return Auto(Length / 6, InputPoints);
    }
    else
    {
        Log::Error("File does not exist, using Dud.txt");
        ifstream DudFile("Dud.txt");
        vector<double> InputDudPoints;
        getline(DudFile, NumberInput);
        double Length = stod(NumberInput);
        for(int i = 0; i < Length * NumPoints; i++)
        {
            getline(DudFile, NumberInput);
            InputDudPoints.push_back(stod(NumberInput));
        }
        return Auto(Length / 6, InputDudPoints);
    }
}

#endif /* UTIL_LinePaths_H_ */