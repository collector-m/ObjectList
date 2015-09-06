#ifndef DETECTBOX_H_INCLUDED
#define DETECTBOX_H_INCLUDED

#include "Point.h"
#include "Line.h"
#include "Object.h"

class DetectBox
{
public:
    DetectBox();
    DetectBox(Point ld_pt, Point rd_pt, Point lu_pt, Point ru_pt);
    ~DetectBox();

    Point local_ld_pt, local_rd_pt, local_lu_pt, local_ru_pt;
    Point frozen_ld_pt, frozen_rd_pt, frozen_lu_pt, frozen_ru_pt;
    Line local_left_line, local_right_line, local_up_line, local_down_line;
    Line frozen_left_line, frozen_right_line, frozen_up_line, frozen_down_line;

    void setDetectBox(Point ld_pt, Point rd_pt, Point lu_pt, Point ru_pt, module::Pose_t current_pose);
    void setDetectBox(module::Pose_t current_pose);

    bool InDetectBox(const Point& local_pt);
    bool InDetectBox(const Object& local_obj);

    Point getFrozenPoint(const Point& pt, module::Pose_t current_pose);
    Point getLocalPoint(const Point& pt, module::Pose_t current_pose);
};

#endif // DETECTBOX_H_INCLUDED