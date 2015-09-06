#ifndef POINT_H_INCLUDED
#define POINT_H_INCLUDED

#include <module/shm.h>
#include <module/module.h>

#include <cmath>
#include <iostream>

using namespace std;

class Point
{
public:
    Point(double x = 0, double y = 0, double dist = 0);
    void setPoint(double x, double y);

    //variables
    double dist;
    double x, y;

private:

};

#endif // POINT_H_INCLUDED
