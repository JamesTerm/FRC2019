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

#define NumPoints 4

struct Point
{
    double X;
    double Y;
    double Act;
    double Speed;
};

struct Auto
{
    Auto(int MaxPoints, vector<double> Points)
    {
        Waypoints = new Point[MaxPoints];
        for(int i = 0; i < MaxPoints; i+=NumPoints)
        {
            Waypoints[i].X = Points[i];
            Waypoints[i].Y = Points[i + 1];
            Waypoints[i].Act = Points[i + 2];
            Waypoints[i].Speed = Points[i + 3];
        }
        Num = MaxPoints;
    }
    Auto()
    {
        Num = 0;
        Waypoints = new Point[0];
    }
    int Num = 0;
    Point* Waypoints;
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
        for(int i = 0; i < Length; i++)
        {
            getline(Inputfile, NumberInput);
            InputPoints.push_back(stod(NumberInput));
        }
        return Auto(Length, InputPoints);
    }
    else
    {
        Log::Error("File does not exist");
        return Auto();
    }
}

#endif /* UTIL_LinePaths_H_ */