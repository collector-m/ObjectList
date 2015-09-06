#ifndef LINE_H_INCLUDED
#define LINE_H_INCLUDED

#include "Point.h"

class Line
{
public:
//functions
    Line();
    Line(Point p1, Point p2);
    void setLine(Point p1, Point p2);

    bool getk() const;    //if k<=1, return true, else return false;
    double getX(double y) const;
    double getY(double x) const;

//variables
    Point pt1, pt2;
    double a, b, c; //ax + by + c = 0;
};

#endif // LINE_H_INCLUDED
