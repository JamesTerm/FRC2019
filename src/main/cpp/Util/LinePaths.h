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

#define NumPoints 3

struct Point
{
    double Act;
    double Angle;
    double Radius;
};

struct Auto
{
    Auto(int MaxPoints, vector<double> Points, bool Offset = false)
    {
        double Xoff = 0;
        double Yoff = 0;
        if (Points.size() % 3 == 0)
        {
            if (Offset)
            {
                Xoff = Points[0];
                Yoff = Points[1];
            }
            for(int i = 0; i < MaxPoints * NumPoints; i+=NumPoints)
            {
                Point* CheckPoint = new Point();
                CheckPoint->Angle = Points[i];// - Xoff;
                CheckPoint->Radius = Points[i + 1];// - Yoff;
                CheckPoint->Act = Points[i + 2];
                Waypoints.push_back(CheckPoint);
            }
            Num = MaxPoints;
        }
        else
        {
            Point* CheckPoint = new Point();
            Waypoints.push_back(CheckPoint);
        }
    }
    int Num = 0;
    vector <Point*> Waypoints;
};

static vector<double> backupPath1()
{
    vector<double> InputBackupPoints;
    string BackupInputString = "5 0 0 0 10 0 0 10 10 0 0 10 0 0 0 0 ";
    istringstream ss(BackupInputString);
    string word;
    int index = 0;
    int Pointsnum = 0;
    while (ss >> word) 
    {
        if (index == 0)
        {
            Pointsnum = stod(word);
        }
        else
        {
            InputBackupPoints.push_back(stod(word));
        }
        index++;
    }
    return InputBackupPoints;
}

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
        for(int i = 0; i < (Length * NumPoints); i++)
        {
            getline(Inputfile, NumberInput);
            InputPoints.push_back(stod(NumberInput));
            Log::General("Line: " + to_string(i) + " = " + to_string(stod(NumberInput)));
        }
        return Auto(Length, InputPoints);
    }
    else
    {
        ifstream DudFile("Dud.txt");
        if (DudFile.is_open())
        {
            Log::Error("File does not exist, using Dud.txt");
            vector<double> InputDudPoints;
            getline(DudFile, NumberInput);
            double Length = stod(NumberInput);
            for(int i = 0; i < Length * NumPoints; i++)
            {
                getline(DudFile, NumberInput);
                InputDudPoints.push_back(stod(NumberInput));
            }
            return Auto(Length, InputDudPoints);
        }
    }
    Log::Error("Files don't not exist, using backup Path");
    vector<double> InputBack = backupPath1();
    return Auto(InputBack.size() / NumPoints, InputBack, true);
}

#endif /* UTIL_LinePaths_H_ */