/****************************** Header ******************************\
Class Name: -
File Name: LinePaths.h
Summary: File of paths for auto use.
Project: FRC2020
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Ian Poll
Email: irobot9803@gmail.com
\********************************************************************/

#pragma once

#ifndef SRC_UTIL_LINEPATHS_H_
#define SRC_UTIL_LINEPATHS_H_

using namespace std;
using namespace Util;

#define MaxPoints 10
#define MaxAutos 3

struct Point
{
    double X;
    double Y;
    double Act;
};

struct Auto
{
    Auto(double** Points)
    {
        int i = 0;
        for(i; i < sizeof(Points) / sizeof(*Points); i++)
        {
            Waypoints[i].X = Points[i][0];
            Waypoints[i].Y = Points[i][1];
            Waypoints[i].Act = Points[i][2];
        }
        int Last = i;
        for(i; i < MaxPoints; i++)
        {
            Waypoints[i].X = Points[Last][0];
            Waypoints[i].Y = Points[Last][1];
            Waypoints[i].Act = Points[Last][2];
        }
    }
    Point *Waypoints = new Points[MaxPoints];
};
/*
static Auto Position1PathNum(int Path)
{
    static Auto Paths[MaxAutos] = {Auto({{0,0,0}})
                                  ,Auto({{0,0,0}})
                                  ,Auto({{0,0,0}})};

    return Paths[Path];
}
*/
#endif /* UTIL_LinePaths_H_ */