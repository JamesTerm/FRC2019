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

#define MaxPoints 10
#define NumPoints 3
#define MaxAutos 3

struct Point
{
    double X;
    double Y;
    double Act;
};

struct Auto
{
    Auto(double Points[MaxPoints][NumPoints])
    {
        for(int i = 0; i < MaxPoints; i++)
        {
            Waypoints[i].X = Points[i][0];
            Waypoints[i].Y = Points[i][1];
            Waypoints[i].Act = Points[i][2];
        }
    }
    int Num = MaxPoints;
    Point *Waypoints = new Point[MaxPoints];
};

static Auto Position1PathNum(int Path)
{
    static double Path1[][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {/*5.8*/0, 5, 0}};
    static double Path2[][3] = {{0, 0, 0}, {1, 1, 0}, {5.8, 5, 0}, {6.4, 5.5, 0}, {8.5, 5.5, 0}, {9, 5.5, 2}, {14, 5.5, 2}, {15, 5, 2}, {15, 4.5, 3}, {15, 4, 1}};
    static double Path3[][3] = {{0, 0, 0}, {1, 1, 0}, {5.8, 5, 0}, {6.4, 5.5, 0}, {8.5, 5.5, 0}, {9, 5.5, 2}, {14, 5.5, 2}, {15, 5, 2}, {15, 4.5, 3}, {15, 4, 1}};
    static Auto Paths[MaxAutos] = {Auto(Path1)
                                  ,Auto(Path2)
                                  ,Auto(Path3)};

    return Paths[Path];
}

#endif /* UTIL_LinePaths_H_ */