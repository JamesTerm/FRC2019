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

#include <map>

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
                CheckPoint->Angle = Points[i] - Xoff;
                CheckPoint->Radius = Points[i + 1] - Yoff;
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

static vector<double> backupPath(string Path)
{
    vector<double> InputBackupPoints;
    std::map<string, string> BackUpPathsDic = {
        {"BackFourth_XYAct", "63 -3.523352 -1.277473 0 -3.47916 -1.124078 0 -3.437703 -0.9620392 0 -3.398297 -0.7935166 0 -3.360259 -0.6206717 0 -3.322904 -0.4456657 0 -3.28555 -0.2706596 0 -3.247511 -0.09781468 0 -3.208106 0.07070804 0 -3.166649 0.2327471 0 -3.122457 0.3861413 0 -3.074846 0.5287298 0 -3.023132 0.6583513 0 -2.966632 0.7728446 0 -2.904662 0.8700483 0 -2.836539 0.9478021 0 -2.707391 1.040225 0 -2.56072 1.095509 0 -2.400539 1.117216 0 -2.230861 1.108907 0 -2.0557 1.074142 0 -1.87907 1.016484 0 -1.704983 0.9394926 0 -1.537453 0.8467305 0 -1.380494 0.7417583 0 -1.285091 0.6594409 0 -1.191951 0.5561387 0 -1.100743 0.4353958 0 -1.011137 0.300756 0 -0.9228024 0.1557643 0 -0.835409 0.003964663 0 -0.7486264 -0.1510988 0 -0.662124 -0.3058821 0 -0.5755717 -0.4568408 0 -0.4886388 -0.6004307 0 -0.4009948 -0.7331082 0 -0.3123097 -0.851328 0 -0.2222532 -0.9515467 0 -0.1304947 -1.03022 0 0.03505145 -1.138111 0 0.2057219 -1.225854 0 0.3793435 -1.289827 0 0.5537431 -1.326408 0 0.7267476 -1.331975 0 0.896184 -1.302906 0 1.059879 -1.23558 0 1.215659 -1.126374 0 1.290169 -1.052016 0 1.362734 -0.9614179 0 1.433555 -0.8562579 0 1.502833 -0.7382169 0 1.57077 -0.608975 0 1.637567 -0.4702122 0 1.703424 -0.3236087 0 1.768544 -0.1708447 0 1.833127 -0.01360035 0 1.897375 0.1464442 0 1.961489 0.307609 0 2.02567 0.4682136 0 2.090119 0.6265781 0 2.155037 0.7810222 0 2.220626 0.9298658 0 2.287088 1.071429 0 "}
    };
    Log::General("************************************Using Path: " + Path + "********************************");
    if(BackUpPathsDic.find(Path) != BackUpPathsDic.end())
    {
        istringstream ss(BackUpPathsDic.find(Path)->second);
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
    vector<double> InputBack = backupPath(Path.substr(0, Path.length() - 4));
    return Auto(InputBack.size() / NumPoints, InputBack, true);
}

#endif /* UTIL_LinePaths_H_ */