#include "Point.h"

Point::Point(double x, double y, double dist)
{
    this->x = x;
    this->y = y;
    this->dist = dist;
    //cout << "Point's constructor called!" << endl;
    //cout << "x: " << x << " y:" << y << endl;
}

//Point::Point(const Point& p)
//{
//    x = p.x;
//    y = p.y;
//    dist = p.dist;
//}
//
//Point& Point::operator=(const Point& p)
//{
//    x = p.x;
//    y = p.y;
//    dist = p.dist;
//}
//
//Point::~Point()
//{
//
//}

void Point::setPoint(double x, double y)
{
    this->x = x;
    this->y = y;
    this->dist = sqrt(pow(x,2) + pow(y,2));
}
