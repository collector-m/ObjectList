#ifndef OBJECTLIST_H_INCLUDED
#define OBJECTLIST_H_INCLUDED

#include <unistd.h>
#include <sys/time.h>
#include <fstream>

#include "Point.h"
#include "Line.h"
#include "Object.h"
#include "pos.h"
#include "DetectBox.h"

const double threshold_corr = 60.0;
const int min_belief = 3;
const int max_beleif = 10;

typedef struct rm_object
{
    //double xmin, ymin, xmax, ymax;
    boundingBox bbx;
} rm_object_t;

const double pi = 3.1415926535;

class ObjectList
{
public:
    ObjectList();
    ~ObjectList();

    //objectlist
    map<unsigned int, Object> objs;
    map<unsigned int, Object> new_objs;
    map<unsigned int, Object> old_objs;

    //detect box
    DetectBox db, ugv_box;
    DetectBox ver_db, horLeft_db, horRight_db;
    DetectBox left_db, right_db;

    bool first_record;          //flag
    module::Pose_t map_center;  //map center GPS
    struct timeval tv;          //time

    void process();
    void processTest();

    bool getSHMBasePoint();
        module::MarkerData_t m_data_active_pt;
        module::Pose_t base_pose, frozen_pose;

    bool getSHMNaviData();
        module::MetaData_t m_data_local_navi;
        module::MetaData_t m_data_navi;
        module::Pose_t current_pose, delta_pose;

    void setDetectBox();

    bool getLocation();
        pos ugv_pos;       //the location of UGV in global map
        Point ugv_pt;

    bool getSHMOccupyGrids();
        //variables
        module::RecoData_t m_recodata_occupyGrids;
        module::RecoOccupyGrids_t m_occupyGrids;
        vector<cluster_pos_t> convex_pos;

    bool setSHMObjectList();
        //variables
        module::RecoData_t m_objectList;

    bool clusterOccupyGrids();
		bool clusterOccupyGrids_old();
        //variables
        map<unsigned int, vector<cluster_pos_t> > clusters;
        map<unsigned int, vector<Point> > cluster_pts;

        //functions
        bool IsAdjacent(cluster_pos_t& cp1, cluster_pos_t& cp2);
        unsigned int getRootID(unsigned int i);
        void mergeGrids(cluster_pos_t& cp1, cluster_pos_t& cp2);

    void setNewObjList(unsigned int cluster_min_size = 1);
        Object setNewObject(const vector<Point>& pts, int line_pairs = 6);
            Point getIntersectPoint(const Line& line1, const Line& line2);
            Point getIntersectPoint(double a1, double b1, double c1, double a2, double b2, double c2);
            boundingBox getBoxVertex(int long_line, Point long_pt1, Point long_pt2, Point short_pt1, Point short_pt2, int line_pairs = 6);
            double getPointDist(const Point& pt1, const Point& pt2);
            Point getAnchorPoint_v1(const boundingBox& bbx);
            Point getAnchorPoint(const boundingBox& bbx);
                bool HaveIntersectPoint(const Line& ugv_line, const Line& box_line);
                    //used for test
                    vector<Point> insection_pts;
                    vector<Line> anchor_lines;

        Object cv_setNewObject(const vector<Point>& pts);

    void setOldObjList();
    void trackObjList();
        //variables
        map<unsigned int, unsigned int> matched_pairs1, matched_pairs2;
        //functions
        int getCorrlation(const Object& old_obj, const Object& new_obj, float location_weight = 0.7, float shape_weight = 0.3);
            int getShapeCorrlation(const Object& old_obj, const Object& new_obj);
            int getLocationCorrlation(const Object& old_obj, const Object& new_obj);
        unsigned char getObjectType(bool BaseOnShift, const Point& shift,
                                    double dynamic_shift = 0.50, double static_shift = 0.13,
                                    double dynamic_velocity = 0.8, double static_velocity = 0.3);
		
		unsigned char getObjectType_v1(bool BaseOnShift, const Point& shift,
                                    double dynamic_shift = 0.5, double static_shift = 0.2,
                                    double dynamic_velocity = 0.8, double static_velocity = 0.3);
        Object updateMatchedObject(const Object& old_obj, const Object& new_obj);
        Object updateOldListObject(const Object& old_obj);
        Object updateNewListObject(const Object& new_obj);


    //used for visualization
    void transOccupyGrids(const vector<cluster_pos_t>& global_convex_pos);
        cluster_pos_t getLocalPos(cluster_pos_t global_pos);
        vector<cluster_pos_t> local_convex_pos;

    void transObjects(const map<unsigned int, Object>& objs);
        Point getLocalPoint(Point global_pt);
		Point getUGVLocalPoint(Point local_pt);
        boundingBox getLocalBoundingBox(const boundingBox& global_bbx);
        map<unsigned int, Object> local_objs;
		
		bool HaveClosestObj;
		Object closest_obj;
	
	void setPathFlag();
	void setTrackObstacle();
		module::obstacle_t track_obs;
	void saveObjects(string filename);
		ofstream outfile;
		unsigned long frame;
	
	void luxstop();
	
    //used for process(test)
    void getTestOccupyGrids();
        vector<Point> click_pts;

    bool sharpenClusters();
        rm_object_t getBoundingBox(const vector<Point>& pts, int line_pairs = 6);
        void sortPoints(boundingBox& bbx);
};

#endif // OBJECTLIST_H_INCLUDED
