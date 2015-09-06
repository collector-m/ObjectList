#include "DetectBox.h"

DetectBox::DetectBox()
{

}

DetectBox::~DetectBox()
{

}

DetectBox::DetectBox(Point ld_pt, Point rd_pt, Point lu_pt, Point ru_pt)
{
    this->local_ld_pt = ld_pt;
    this->local_rd_pt = rd_pt;
    this->local_lu_pt = lu_pt;
    this->local_ru_pt = ru_pt;
}

void DetectBox::setDetectBox(Point ld_pt, Point rd_pt, Point lu_pt, Point ru_pt, module::Pose_t current_pose)
{
    this->local_ld_pt = ld_pt;
    this->local_rd_pt = rd_pt;
    this->local_lu_pt = lu_pt;
    this->local_ru_pt = ru_pt;

    this->frozen_ld_pt = getFrozenPoint(ld_pt, current_pose);
    this->frozen_rd_pt = getFrozenPoint(rd_pt, current_pose);
    this->frozen_lu_pt = getFrozenPoint(lu_pt, current_pose);
    this->frozen_ru_pt = getFrozenPoint(ru_pt, current_pose);

    this->local_left_line.setLine(ld_pt, lu_pt);
    this->local_right_line.setLine(rd_pt, ru_pt);
    this->local_down_line.setLine(ld_pt, rd_pt);
    this->local_up_line.setLine(ld_pt, rd_pt);

    this->frozen_left_line.setLine(frozen_ld_pt, frozen_lu_pt);
    this->frozen_right_line.setLine(frozen_rd_pt, frozen_ru_pt);
    this->frozen_down_line.setLine(frozen_ld_pt, frozen_rd_pt);
    this->frozen_up_line.setLine(frozen_ld_pt, frozen_rd_pt);
}

void DetectBox::setDetectBox(module::Pose_t current_pose)
{
    this->frozen_ld_pt = getFrozenPoint(local_ld_pt, current_pose);
    this->frozen_rd_pt = getFrozenPoint(local_rd_pt, current_pose);
    this->frozen_lu_pt = getFrozenPoint(local_lu_pt, current_pose);
    this->frozen_ru_pt = getFrozenPoint(local_ru_pt, current_pose);

    this->frozen_left_line.setLine(frozen_ld_pt, frozen_lu_pt);
    this->frozen_right_line.setLine(frozen_rd_pt, frozen_ru_pt);
    this->frozen_down_line.setLine(frozen_ld_pt, frozen_rd_pt);
    this->frozen_up_line.setLine(frozen_ld_pt, frozen_rd_pt);
}

Point DetectBox::getFrozenPoint(const Point& pt, module::Pose_t current_pose)
{
    Point fpt;
    double x = cos(current_pose.eulr) * pt.x + sin(current_pose.eulr) * pt.y;
    double y = -sin(current_pose.eulr) * pt.x + cos(current_pose.eulr) * pt.y;
    fpt.setPoint(x, y);
    return fpt;
}

Point DetectBox::getLocalPoint(const Point& pt, module::Pose_t current_pose)
{
    Point lpt;
    double x = cos(current_pose.eulr) * pt.x - sin(current_pose.eulr) * pt.y;
    double y = sin(current_pose.eulr) * pt.x + cos(current_pose.eulr) * pt.y;
    lpt.setPoint(x, y);
    return lpt;
}

bool DetectBox::InDetectBox(const Point& local_pt)
{
    if(local_pt.x >= local_ld_pt.x && local_pt.x <= local_rd_pt.x
            && local_pt.y >= local_ld_pt.y && local_pt.y <= local_lu_pt.y)
        return true;
    else
        return false;
}

bool DetectBox::InDetectBox(const Object& obj)
{
    if(InDetectBox(obj.ugv_anchor_pt))
        return true;
    else
        return false;
}
