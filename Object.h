#ifndef OBJECT_H_INCLUDED
#define OBJECT_H_INCLUDED

#include "Point.h"
#include <deque>

typedef struct _boundingBox
{
    Point vertex[4];
} boundingBox;

typedef struct _objInfo
{
    struct timeval tv;
    Point lc;
} objInfo;

class Object
{
public:
    Object();
    ~Object();

    unsigned char type;
    //type:
    //  0. unknown
    //  1. static
    //  2. dynamic
    boundingBox bbx;
    Point anchor_pt, ugv_anchor_pt;

    short belief;
    double length, width;
    int pd;

    //functions
    void updateObjInfoDeque(const Point& current_lc, timeval tv, unsigned int deque_size = 20);
        deque<objInfo> obj_lcs;
    void setObjInfo();                  //set obj_shifts, sum_shift and velocity
        vector<objInfo> obj_shifts;
        Point sum_shift;
        Point velocity;

    //void updateShiftDeque(const Point& current_position, unsigned int deque_size = 10);
    //deque<Point> obj_position;
    //Point getShift();
    //Point shift;
};


#endif // OBJECT_H_INCLUDED
