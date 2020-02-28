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
    static double Path1[][3] = {{0, 0, 0}, {6.55, 0, 0}, {4.57, 8.63, 0}, {0.25, 11.08, 0}, {-2.48, 11.08, 0}, {3.13, 16.37, 0}, {2.13, 16.37, 0}, {17.62, 16.37, 0}, {7.48, 11.2, 0}, {-2.1, 11.2, 0}};
    static double Path2[][3] = {{0, 0, 0}, {6.55, 0, 0}, {4.57, 8.63, 0}, {0.25, 11.08, 0}, {-2.48, 11.08, 0}, {3.13, 16.37, 0}, {2.13, 16.37, 0}, {17.62, 16.37, 0}, {7.48, 11.2, 0}, {-2.1, 11.2, 0}};
    static double Path3[][3] = {{0, 0, 0}, {6.55, 0, 0}, {4.57, 8.63, 0}, {0.25, 11.08, 0}, {-2.48, 11.08, 0}, {3.13, 16.37, 0}, {2.13, 16.37, 0}, {17.62, 16.37, 0}, {7.48, 11.2, 0}, {-2.1, 11.2, 0}};
    static Auto Paths[MaxAutos] = {Auto(Path1)
                                  ,Auto(Path2)
                                  ,Auto(Path3)};

    return Paths[Path];
}

#endif /* UTIL_LinePaths_H_ */