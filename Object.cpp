#include "Object.h"

Object::Object()
{
    type = 0;
    belief = 0;
    length = 0, width = 0;
    pd = 0;
}

Object::~Object()
{
    obj_lcs.clear();
}

void Object::setObjInfo()
{
    obj_shifts.clear();
    if(obj_lcs.size() > 0) {
        for(deque<objInfo>::iterator deque_iter1 = obj_lcs.begin();
                deque_iter1 != obj_lcs.end() - 1; deque_iter1++) {
            deque<objInfo>::iterator deque_iter2 = deque_iter1 + 1;
            objInfo tmp_shift;
            tmp_shift.tv.tv_sec = deque_iter2->tv.tv_sec - deque_iter1->tv.tv_sec;
            tmp_shift.tv.tv_usec = deque_iter2->tv.tv_usec - deque_iter1->tv.tv_usec;
            tmp_shift.lc.x = deque_iter2->lc.x - deque_iter1->lc.x;
            tmp_shift.lc.y = deque_iter2->lc.y - deque_iter1->lc.y;
            obj_shifts.push_back(tmp_shift);
        }
    }
    sum_shift.x = 0, sum_shift.y = 0;
    Point sum_velocity;
    sum_velocity.x = 0, sum_velocity.y = 0;
    for(vector<objInfo>::const_iterator shift_iter = obj_shifts.begin();
            shift_iter != obj_shifts.end(); shift_iter++) {
        //cout << "interval time: " << shift_iter->tv.tv_sec + shift_iter->tv.tv_usec / 1000000.0 << endl;
        sum_shift.x += shift_iter->lc.x;
        sum_shift.y += shift_iter->lc.y;
        double time_interval = shift_iter->tv.tv_sec + (shift_iter->tv.tv_usec / 1000000.0);
        sum_velocity.x += shift_iter->lc.x / time_interval;
        sum_velocity.y += shift_iter->lc.y / time_interval;
    }
    if(obj_shifts.size() > 0) {
        velocity.x = sum_velocity.x / obj_shifts.size();
        velocity.y = sum_velocity.y / obj_shifts.size();
    } else {
        velocity.x = 0;
        velocity.y = 0;
    }

}

void Object::updateObjInfoDeque(const Point& current_lc, timeval tv, unsigned int deque_size)
{
    objInfo tmp_info;
    tmp_info.lc = current_lc;
    tmp_info.tv = tv;
    obj_lcs.push_back(tmp_info);
    if(obj_lcs.size() > deque_size)
        obj_lcs.pop_front();
}

//void Object::updateShiftDeque(const Point& current_position, unsigned int deque_size)
//{
//    obj_position.push_back(current_position);
//    if(obj_position.size() > deque_size)
//        obj_position.pop_front();
//}

//Point Object::getShift()
//{
//    //input: deque<Point> obj_position
//    //output: Point shift
//    Point shift;
//    shift.x = 0;
//    shift.y = 0;
//    for(deque<Point>::iterator deque_iter1 = obj_position.begin();
//            deque_iter1 != obj_position.end() - 1; deque_iter1++) {
//        deque<Point>::iterator deque_iter2 = deque_iter1 + 1;
//        shift.x += deque_iter1->x - deque_iter2->x;
//        shift.y += deque_iter1->y - deque_iter2->y;
//    }
//    return shift;
//}
