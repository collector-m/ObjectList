#include "Line.h"

Line::Line()
{

}

Line::Line(Point p1, Point p2)
{
    this->pt1 = p1;
    this->pt2 = p2;
    double delta_x = p2.x - p1.x;
    double delta_y = p2.y - p1.y;
	if(abs(delta_x) >= abs(delta_y)) {
		double k = delta_y / delta_x;
		double t = p1.y - k * p1.x;
		a = k;
		b = -1;
		c = t;
	} else {
        double kk = delta_x / delta_y;
        double tt = p1.x - kk * p1.y;
        a = 1;
        b = -1 * kk;
        c = -1 * tt;
	}
}

bool Line::getk() const
{
    if(abs(a) > abs(b))
		return false;//|k|>1
	else
		return true;//|k|<=1
}

double Line::getX(double y) const
{
    double x;
	x = (c + b*y)/(-1 * a);
	return x;
}

double Line::getY(double x) const
{
    double y;
	y = (c + a*x)/(-1 * b);
	return y;
}

void Line::setLine(Point p1, Point p2)
{
    this->pt1 = p1;
    this->pt2 = p2;
    double delta_x = p2.x - p1.x;
    double delta_y = p2.y - p1.y;
	if(abs(delta_x) >= abs(delta_y)) {
		double k = delta_y / delta_x;
		double t = p1.y - k * p1.x;
		a = k;
		b = -1;
		c = t;
	} else {
        double kk = delta_x / delta_y;
        double tt = p1.x - kk * p1.y;
        a = 1;
        b = -1 * kk;
        c = -1 * tt;
	}
}
