#include "ObjectList.h"

ObjectList::ObjectList()
{
	first_record = true;

	//kunchenghu
	map_center.x = 418758.0;//418758.4;//422172.4;//422172.4;
	map_center.y = -917173.0;//-917174.2;//-918693.6;// -918732.5;
	map_center.eulr = 0;

	//jiugongge
	//map_center.x = 422172.4;
	//map_center.y = -918693.6;
	//map_center.eulr = 0;

	//base pose
	m_data_active_pt.value.v_activepointinroute.pre_pt.x = map_center.x;
	m_data_active_pt.value.v_activepointinroute.pre_pt.y = map_center.y;
	m_data_active_pt.value.v_activepointinroute.pre_pt.eulr = map_center.eulr;

	base_pose.x = map_center.x;
	base_pose.y = map_center.y;
	base_pose.eulr = map_center.eulr;

	//map center in frozen coordinate pose
	frozen_pose.x = map_center.x - base_pose.x;
	frozen_pose.y = map_center.y - base_pose.y;
	frozen_pose.eulr = map_center.eulr - base_pose.eulr;

	//current pose in frozen coordinate
	current_pose.x = 0;
	current_pose.y = 0;
	current_pose.eulr = 0;

	delta_pose.x = 0;
	delta_pose.y = 0;
	delta_pose.eulr = 0;

	Point ugv_ld_pt, ugv_rd_pt, ugv_lu_pt, ugv_ru_pt;
	ugv_ld_pt.setPoint(-1.2, -2.5);
	ugv_rd_pt.setPoint( 1.2, -2.5);
	ugv_lu_pt.setPoint(-1.2,  2.5);
	ugv_ru_pt.setPoint( 1.2,  2.5);
	ugv_box.setDetectBox(ugv_ld_pt, ugv_rd_pt, ugv_lu_pt, ugv_ru_pt, current_pose);

	Point db_ld_pt, db_rd_pt, db_lu_pt, db_ru_pt;
	db_ld_pt.setPoint(-10.0,  0.0);
	db_rd_pt.setPoint( 10.0,  0.0);
	db_lu_pt.setPoint(-10.0, 80.0);
	db_ru_pt.setPoint( 10.0, 80.0);
	db.setDetectBox(db_ld_pt, db_rd_pt, db_lu_pt, db_ru_pt, current_pose);

	Point vdb_ld_pt, vdb_rd_pt, vdb_lu_pt, vdb_ru_pt;
	vdb_ld_pt.setPoint(-1.2,  0.0);
	vdb_rd_pt.setPoint( 1.2,  0.0);
	vdb_lu_pt.setPoint(-1.2, 80.0);
	vdb_ru_pt.setPoint( 1.2, 80.0);
	ver_db.setDetectBox(vdb_ld_pt, vdb_rd_pt, vdb_lu_pt, vdb_ru_pt, current_pose);

	Point hldb_ld_pt, hldb_rd_pt, hldb_lu_pt, hldb_ru_pt;
	hldb_ld_pt.setPoint(-10.0,  0.0);
	hldb_rd_pt.setPoint( -1.2,  0.0);
	hldb_lu_pt.setPoint(-10.0, 20.0);
	hldb_ru_pt.setPoint( -1.2, 20.0);
	horLeft_db.setDetectBox(hldb_ld_pt, hldb_rd_pt, hldb_lu_pt, hldb_ru_pt, current_pose);

	Point hrdb_ld_pt, hrdb_rd_pt, hrdb_lu_pt, hrdb_ru_pt;
	hrdb_ld_pt.setPoint(  1.2,  0.0);
	hrdb_rd_pt.setPoint( 10.0,  0.0);
	hrdb_lu_pt.setPoint(  1.2, 20.0);
	hrdb_ru_pt.setPoint( 10.0, 20.0);
	horRight_db.setDetectBox(hrdb_ld_pt, hrdb_rd_pt, hrdb_lu_pt, hrdb_ru_pt, current_pose);

	Point ldb_ld_pt, ldb_rd_pt, ldb_lu_pt, ldb_ru_pt;
	ldb_ld_pt.setPoint(-3.6,  0.0);
	ldb_rd_pt.setPoint(-1.2,  0.0);
	ldb_lu_pt.setPoint(-3.6, 80.0);
	ldb_ru_pt.setPoint(-1.2, 80.0);
	left_db.setDetectBox(ldb_ld_pt, ldb_rd_pt, ldb_lu_pt, ldb_ru_pt, current_pose);

	Point rdb_ld_pt, rdb_rd_pt, rdb_lu_pt, rdb_ru_pt;
	rdb_ld_pt.setPoint( 1.2,  0.0);
	rdb_rd_pt.setPoint( 3.6,  0.0);
	rdb_lu_pt.setPoint( 1.2, 80.0);
	rdb_ru_pt.setPoint( 3.6, 80.0);
	right_db.setDetectBox(rdb_ld_pt, rdb_rd_pt, rdb_lu_pt, rdb_ru_pt, current_pose);

	frame = 0;
}

ObjectList::~ObjectList()
{
	objs.clear();
	new_objs.clear();
	old_objs.clear();
	clusters.clear(), cluster_pts.clear();
	matched_pairs1.clear(), matched_pairs2.clear();
}

void ObjectList::process()
{
	getSHMNaviData();
	getLocation();
	setDetectBox();
	if(getSHMOccupyGrids()) {
		clusterOccupyGrids();
		setNewObjList();
		trackObjList();

		transOccupyGrids(convex_pos);
		transObjects(objs);

		setSHMObjectList();

		setPathFlag();
		setTrackObstacle();
		luxstop();

		frame++;
	} else {
		cout << "OCCUPY GRIDS ARE OOOOOOOOOOOOOOOOOOOOOOOOLD!" << endl;
		usleep(1000*10);    //10ms
	}
}

void ObjectList::processTest()
{
	getSHMNaviData();
	getLocation();
	setDetectBox();
	getTestOccupyGrids();
	clusterOccupyGrids();
	setNewObjList();
	trackObjList();

	transOccupyGrids(convex_pos);
	transObjects(objs);
	setSHMObjectList();
}

bool ObjectList::getSHMBasePoint()
{
	//read base pose
	m_data_active_pt.type = module::MarkerData::MARKER_ACTIVEPOINTINROUTE;
	module::shm::SHARED_OBJECTS.GetMarker(&m_data_active_pt);
	base_pose.x = m_data_active_pt.value.v_activepointinroute.pre_pt.x;
	base_pose.y = m_data_active_pt.value.v_activepointinroute.pre_pt.y;
	base_pose.eulr = m_data_active_pt.value.v_activepointinroute.pre_pt.eulr;
	cout << "Base_pose: " << "  x:  " << base_pose.x << "  y:  " << base_pose.y << "  eulr:  " << base_pose.eulr << endl;

	//base_pose.x = -12645.2, base_pose.y = -13982.2;

	frozen_pose.x = map_center.x - base_pose.x;
	frozen_pose.y = map_center.y - base_pose.y;
	frozen_pose.eulr = map_center.eulr - base_pose.eulr;

	//frozen_pose.x = 0;
	//frozen_pose.y = 0;
	//frozen_pose.eulr = 0;

	cout << " The global map center's frozen position: " << "  x:  " << frozen_pose.x << "  y:  "
	     << frozen_pose.y << "  eulr:  " << frozen_pose.eulr << endl;
	return true;
}

bool ObjectList::getSHMNaviData()
{
	m_data_local_navi.type = module::MetaData::META_LOCAL_NAVIGATION;
	module::shm::SHARED_OBJECTS.GetMetaData(&m_data_local_navi);
	current_pose.x =  m_data_local_navi.value.v_navi_local.UGVtoFCF[0];
	current_pose.y =  m_data_local_navi.value.v_navi_local.UGVtoFCF[1];
	current_pose.eulr =  m_data_local_navi.value.v_navi_local.UGVtoFCF[2];

	current_pose.x += delta_pose.x;
	current_pose.y += delta_pose.y;
	current_pose.eulr += delta_pose.eulr;

	cout << "delta_x: " << delta_pose.x << " delta_y: " << delta_pose.y << " delta_eulr: " << delta_pose.eulr << endl;
	cout << "Current pose: " << " x: " << current_pose.x << " y : " << current_pose.y << " eulr: " << current_pose.eulr << endl;
	return true;
}

void ObjectList::setDetectBox()
{
	ugv_box.setDetectBox(current_pose);
	db.setDetectBox(current_pose);
	ver_db.setDetectBox(current_pose);
	horLeft_db.setDetectBox(current_pose);
	horRight_db.setDetectBox(current_pose);
	left_db.setDetectBox(current_pose);
	right_db.setDetectBox(current_pose);
}

bool ObjectList::getLocation()
{
	//This function is used to locate the vehicle on the global map
	//input:  1. Navigation data: current_pose
	//        2. Map center on frozen coordinate: frozen_pose
	//output: 1. ugv_pos
	//        2. ugv_pt
	double diff_x = current_pose.x - frozen_pose.x;
	double diff_y = current_pose.y - frozen_pose.y;
	int diff_row = diff_y / 0.20;
	int diff_col = diff_x / 0.20;

	ugv_pt.x = diff_x;
	ugv_pt.y = diff_y;
	ugv_pos.row = diff_row;
	ugv_pos.col = diff_col;

	cout << "UGV info: " << " x: " << ugv_pt.x << " y: " << ugv_pt.y
	     << " row: " << ugv_pos.row << " col: " << ugv_pos.col << endl;

	return true;
}

bool ObjectList::getSHMOccupyGrids()
{
	//input: m_occupyGrids(read from shm)
	//output: vector<cluster_pos_t> convex_pos;

//    typedef struct occupy_grid
//    {
//        int row;
//        int col;
//    } occupy_grid_t;
//
//    typedef struct RecoOccupyGrids
//    {
//        bool interactive_flag;
//        unsigned int num;
//        occupy_grid_t occupy_grids[MAX_OCCUPY_NUM];
//    } RecoOccupyGrids_t;

	convex_pos.clear();

	//get current time
	gettimeofday(&tv, NULL);

	m_recodata_occupyGrids.type = module::RecoData_t::RT_OCCUPY_GRIDS;
	module::shm::SHARED_OBJECTS.GetRecoData(&m_recodata_occupyGrids);
	m_occupyGrids = m_recodata_occupyGrids.value.v_occupy_grids;
	if(m_occupyGrids.interactive_flag) {
		//RasterMap has been set the newest occupied grids
		for(unsigned int i = 0; i < m_occupyGrids.num; i++) {
			cluster_pos_t tmp_pos;

			tmp_pos.id = i;
			tmp_pos.row = m_occupyGrids.occupy_grids[i].row;
			tmp_pos.col = m_occupyGrids.occupy_grids[i].col;

			convex_pos.push_back(tmp_pos);
		}
		m_recodata_occupyGrids.value.v_occupy_grids.interactive_flag = false;
		module::shm::SHARED_OBJECTS.SetRecoData(m_recodata_occupyGrids);
		return true;
	} else {
		//RasterMap has been set the newest occupied grids has not been set the newest occupied grids
		m_recodata_occupyGrids.value.v_occupy_grids.interactive_flag = false;
		module::shm::SHARED_OBJECTS.SetRecoData(m_recodata_occupyGrids);
		return false;
	}

	//unsigned int convex_pos_cnt = 0;

	return true;
}

bool ObjectList::setSHMObjectList()
{
	//input: objs
	//output: m_objectList

//    #define MAX_OBSTACLE_NUM 5

//    typedef struct obstacle
//    {
//        unsigned char type;			//ÀàÐÍ
//        gridFXY_t anchor_pt;			//Ãªµã
//        gridFXY_t bounding_box[4];		//°üÂçºÐµÄËÄ¸ö¶¥µã
//        gridFXY_t velocity;				//ËÙ¶È
//    } obstacle_t;

//    typedef struct RecoObstacleList
//    {
//        bool interactive_flag;
//        unsigned int num;			//ÕÏ°­ÎïµÄ¸öÊý
//        obstacle_t obs[MAX_OBSTACLE_NUM];	//ÕÏ°­ÎïÊý×é
//    } RecoObstacleList_t;
	m_objectList.type = module::RecoData_t::RT_OBSTACLE_LIST;

	unsigned int obj_cnt = 0;
	unsigned int shm_obj_cnt = 0;
	for(map<unsigned int, Object>::const_iterator map_iter = objs.begin();
	    map_iter != objs.end(); map_iter++) {
		if(map_iter->second.belief >= 8) {
			if(ver_db.InDetectBox(local_objs[map_iter->first])) {

				//cout << "Set the object to SHM!!!" << endl;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].type = map_iter->second.type;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].anchor_pt.x = map_iter->second.anchor_pt.x;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].anchor_pt.y = map_iter->second.anchor_pt.y;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].velocity.x = map_iter->second.velocity.x;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].velocity.y = map_iter->second.velocity.y;
				cout << "Last Anchor pt: " << map_iter->second.anchor_pt.x << " " << map_iter->second.anchor_pt.y << endl;
				//set the bounding box
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].bounding_box[0].x = map_iter->second.bbx.vertex[0].x;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].bounding_box[0].y = map_iter->second.bbx.vertex[0].y;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].bounding_box[1].x = map_iter->second.bbx.vertex[1].x;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].bounding_box[1].y = map_iter->second.bbx.vertex[1].y;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].bounding_box[2].x = map_iter->second.bbx.vertex[2].x;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].bounding_box[2].y = map_iter->second.bbx.vertex[2].y;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].bounding_box[3].x = map_iter->second.bbx.vertex[3].x;
				m_objectList.value.v_obstacle_list.obs[shm_obj_cnt].bounding_box[3].y = map_iter->second.bbx.vertex[3].y;
				shm_obj_cnt++;
			}
		}
		obj_cnt++;
	}
	m_objectList.value.v_obstacle_list.num = shm_obj_cnt;
	m_objectList.value.v_obstacle_list.interactive_flag = true;

	cout << "Have " << obj_cnt << " obstacles and set " << shm_obj_cnt << " obstacles! " << endl;
	module::shm::SHARED_OBJECTS.SetRecoData(m_objectList);
	return true;
}

bool ObjectList::clusterOccupyGrids()
{
	//input: vector<cluster_pos_t> convex_pos;
	//output: map<unsigned int, vector<cluster_pos_t> > clusters;
	//		  map<unsigned int, vector<Point> > cluster_pts;
	clusters.clear();
	cluster_pts.clear();

	unsigned int id1 = 0, id2 = 0;
	//unsigned int sid = big = 0;
	for(vector<cluster_pos_t>::iterator pos_iter = convex_pos.begin();
	    pos_iter != convex_pos.end(); pos_iter++) {
		clusters[pos_iter->id].push_back(*pos_iter);
	}

	if(convex_pos.size() > 1) {
		for(vector<cluster_pos_t>::iterator pos_iter1 = convex_pos.begin();
		    pos_iter1 != convex_pos.end()-1; pos_iter1 ++) {
			for(vector<cluster_pos_t>::iterator pos_iter2 = pos_iter1+1;
			    pos_iter2 != convex_pos.end(); pos_iter2++) {
				if(IsAdjacent(*pos_iter1, *pos_iter2)) {
					id1 = pos_iter1->id;
					id2 = pos_iter2->id;
					if(id1 != id2) {
						if(id1 > id2) {
							pos_iter1->id = id2;
							for(vector<cluster_pos>::iterator cpos_iter = clusters[id1].begin();
							    cpos_iter != clusters[id1].end(); cpos_iter++) {
								cpos_iter->id = id2;
								clusters[id2].push_back(*cpos_iter);
							}
							clusters.erase(id1);
						} else {
							pos_iter2->id = id1;
							for(vector<cluster_pos>::iterator cpos_iter = clusters[id2].begin();
							    cpos_iter != clusters[id2].end(); cpos_iter++) {
								cpos_iter->id = id1;
								clusters[id1].push_back(*cpos_iter);
							}
							clusters.erase(id2);
						}
					}
				}
			}
		}
	}

	unsigned int cluster_cnt = 0;
	for(map<unsigned int, vector<cluster_pos_t> >::const_iterator map_iter = clusters.begin();
	    map_iter != clusters.end(); map_iter ++) {
		//cout << "     cluster " << map_iter->first << " contains grids: " << endl;
		for(vector<cluster_pos_t>::const_iterator pos_iter = map_iter->second.begin();
		    pos_iter != map_iter->second.end(); pos_iter ++ ) {
			//cout << "          " << " id: " << pos_iter->id << " row: " << pos_iter->row << " col: " << pos_iter->col << endl;
			Point tmp_pt;
			//tmp_pt.x = getx(0, getLocalCol(pos_iter->row, pos_iter->col));
			//tmp_pt.y = gety(getLocalRow(pos_iter->row, pos_iter->col), 0);
			tmp_pt.x = pos_iter->col * 0.2;
			tmp_pt.y = pos_iter->row * 0.2;
			cluster_pts[cluster_cnt].push_back(tmp_pt);
		}
		cluster_cnt++;
	}

	//print the cluster info(pos):
//    for(map<unsigned int, vector<cluster_pos_t> >::const_iterator clus_iter = clusters.begin();
//            clus_iter != clusters.end(); clus_iter++) {
//        cout << "cluster " << clus_iter->first << endl;
//        for(vector<cluster_pos_t>::const_iterator pos_iter = clus_iter->second.begin();
//                pos_iter != clus_iter->second.end(); pos_iter++) {
//            cout << "   " << " row: " << pos_iter->row << " col: " << pos_iter->col << endl;
//        }
//    }


	//print the cluster points in(pt)
//	for(map<unsigned int, vector<Point> >::iterator map_iter = cluster_pts.begin();
//            map_iter != cluster_pts.end(); map_iter++) {
//        cout << "cluster " << map_iter->first << endl;
//        for(vector<Point>::const_iterator pt_iter = map_iter->second.begin();
//                pt_iter != map_iter->second.end(); pt_iter++) {
//            cout << "   " << " x: " << pt_iter->x << " y: " << pt_iter->y << endl;
//        }
//    }

	return true;
}


bool ObjectList::clusterOccupyGrids_old()
{
	clusters.clear();
	cluster_pts.clear();
	if(convex_pos.size() > 0) {
		for(vector<cluster_pos_t>::iterator iter1 = convex_pos.begin();
		    iter1 != convex_pos.end() - 1; iter1++) {
			for(vector<cluster_pos_t>::iterator iter2 = iter1 + 1;
			    iter2 != convex_pos.end(); iter2++) {
				if(IsAdjacent(*iter1, *iter2)) {
					mergeGrids(*iter1, *iter2);
				}
			}
		}

		//unsigned int cluster_cnt = 0;
		for(vector<cluster_pos_t>::const_iterator iter = convex_pos.begin();
		    iter != convex_pos.end(); iter++) {
			unsigned int t = getRootID(iter->id);
			clusters[t].push_back(*iter);
			//push the cluster points
			//Point tmp_pt;
			//tmp_pt.x = iter->col * 0.2;
			//tmp_pt.y = iter->row * 0.2;
			//cluster_pts[cluster_cnt].push_back(tmp_pt);
			//cluster_cnt++;
		}
	}

	//print clusters
	//cout << "Print the clusters info: " << endl;
	unsigned int cluster_cnt = 0;
	for(map<unsigned int, vector<cluster_pos_t> >::const_iterator map_iter = clusters.begin();
	    map_iter != clusters.end(); map_iter ++) {
		//cout << "     cluster " << map_iter->first << " contains grids: " << endl;
		for(vector<cluster_pos_t>::const_iterator pos_iter = map_iter->second.begin();
		    pos_iter != map_iter->second.end(); pos_iter ++ ) {
			//cout << "          " << " id: " << pos_iter->id << " row: " << pos_iter->row << " col: " << pos_iter->col << endl;
			Point tmp_pt;
			tmp_pt.x = pos_iter->col * 0.2;
			tmp_pt.y = pos_iter->row * 0.2;
			cluster_pts[cluster_cnt].push_back(tmp_pt);
		}
		cluster_cnt++;
	}

	//print cluster points
	//cout << "Print the cluster points info: " << endl;
	for(map<unsigned int, vector<Point> >::const_iterator map_iter = cluster_pts.begin();
	    map_iter != cluster_pts.end(); map_iter++) {
		//cout << "     cluster " << map_iter->first << " contains points: " << endl;
		for(vector<Point>::const_iterator pt_iter = map_iter->second.begin();
		    pt_iter != map_iter->second.end(); pt_iter++) {
			//cout << "          " << "  x: " << pt_iter->x << "  y: " << pt_iter->y << endl;
		}
	}
	return true;
}

bool ObjectList::IsAdjacent(cluster_pos_t& cp1, cluster_pos_t& cp2)
{
	int row_dist = abs(cp1.row - cp2.row);
	int col_dist = abs(cp1.col - cp2.col);
	if((row_dist == 0) && (col_dist == 0)) {
		return false;
	} else {
		if((row_dist <= 2) && (col_dist <= 2))
			return true;
		else
			return false;
	}
}

unsigned int ObjectList::getRootID(unsigned int i)
{
	while(i != convex_pos[i].id)
		i = convex_pos[i].id;
	return i;
}

void ObjectList::mergeGrids(cluster_pos_t& cp1, cluster_pos_t& cp2)
{
	unsigned int id1 = getRootID(cp1.id);
	unsigned int id2 = getRootID(cp2.id);
	unsigned int id  = min(id1, id2);
	cp1.id = id;
	cp2.id = id;
}

void ObjectList::setNewObjList(unsigned int cluster_min_size)
{
	anchor_lines.clear();
	insection_pts.clear();

	//print cluster points
//    for(map<unsigned int, vector<Point> >::const_iterator map_iter = cluster_pts.begin();
//            map_iter != cluster_pts.end(); map_iter++) {
//        cout << "     cluster " << map_iter->first << " contains points: " << endl;
//        for(vector<Point>::const_iterator pt_iter = map_iter->second.begin();
//                pt_iter != map_iter->second.end(); pt_iter++) {
//            cout << "          " << "  x: " << pt_iter->x << "  y: " << pt_iter->y << endl;
//        }
//    }
	new_objs.clear();
	unsigned int obj_cnt = 0;
	for(map<unsigned int, vector<Point> >::iterator map_iter = cluster_pts.begin();
	    map_iter != cluster_pts.end(); map_iter++) {
		//if the number of points in such cluster is more than 'cluster_min_size',
		//  the cluster will be set to a new object
		if(map_iter->second.size() > cluster_min_size) {
			new_objs[obj_cnt] = setNewObject(map_iter->second);
			obj_cnt++;
		}
	}
	//print new object list info:
	/*
	cout << "New object list: " << endl;
	for(map<unsigned int, Object>::const_iterator map_iter = new_objs.begin();
	        map_iter != new_objs.end(); map_iter++) {
	    cout << "    Object " << map_iter->first << " Length: " << map_iter->second.length << " Width: " << map_iter->second.width
	        << " belief: " << map_iter->second.belief << " type: " << map_iter->second.type << endl;
	    cout << "         vertex[0]: " << "( " << map_iter->second.bbx.vertex[0].x << " , " << map_iter->second.bbx.vertex[0].y << " ) " << endl;
	    cout << "         vertex[1]: " << "( " << map_iter->second.bbx.vertex[1].x << " , " << map_iter->second.bbx.vertex[1].y << " ) " << endl;
	    cout << "         vertex[2]: " << "( " << map_iter->second.bbx.vertex[2].x << " , " << map_iter->second.bbx.vertex[2].y << " ) " << endl;
	    cout << "         vertex[3]: " << "( " << map_iter->second.bbx.vertex[3].x << " , " << map_iter->second.bbx.vertex[3].y << " ) " << endl;
	}
	*/
}

Object ObjectList::setNewObject(const vector<Point>& pts, int line_pairs)
{
	//int line_pairs = 6;
	double delta_angle = (pi / 2) / line_pairs;     //15
	//cout << "delta angle: " << delta_angle / pi * 180 << endl;

	double area_min = 10000.0;
	double length, width;
	int select_line = 0;

	Point long_pt[2], short_pt[2];
	//double a_max, b_max, c_max;
	//a_max = b_max = c_max = 0;

	//select the line pair
	for(int i = 0; i < line_pairs; i++) {
		double angle[2];
		//double tanValue[2];
		double a[2], b[2], c[2];
		angle[0] = i * delta_angle;
		angle[1] = angle[0] + pi / 2;
		//cout << "--------------------------------------------------------" << endl;
		//cout << "angle 0: " <<  angle[0] / pi * 180 << " angle 1: " <<  angle[1] / pi * 180 << endl;
		if(i == 0) {
			a[0] = 0;
			b[0] = 1;
			a[1] = 1;
			b[1] = 0;
		} else {
			a[0] = tan(angle[0]);
			b[0] = -1;
			a[1] = tan(angle[1]);
			b[1] = -1;
		}
		Point head_pt = *(pts.begin());

		c[0] = - a[0]*head_pt.x - b[0]*head_pt.y;

		//Here, pt_set1 and pt_set2 are point sets. Their elements are intersection points
		//First element is the origin point. Second element is the intersection point
		//vector<Point> pt_set1;
		vector<pair<Point, Point> > pt_set1;
		for(vector<Point>::const_iterator pt_iter = pts.begin();
		    pt_iter != pts.end(); pt_iter++) {
			double c1 = - a[1] * pt_iter->x - b[1] * pt_iter->y;
			Point tmp_pt = getIntersectPoint(a[0], b[0], c[0], a[1], b[1], c1);
			//pt_set1.push_back(tmp_pt);
			pair<Point, Point> point_pair = make_pair(*pt_iter, tmp_pt);
			pt_set1.push_back(point_pair);
		}

		c[1] = - a[1] * head_pt.x - b[1] * head_pt.y;
		//vector<Point> pt_set2;
		vector<pair<Point, Point> > pt_set2;
		for(vector<Point>::const_iterator pt_iter = pts.begin();
		    pt_iter != pts.end(); pt_iter++) {
			double c0 = - a[0] * pt_iter->x - b[0] * pt_iter->y;
			Point tmp_pt = getIntersectPoint(a[0], b[0], c0, a[1], b[1], c[1]);
			//pt_set2.push_back(tmp_pt);
			pair<Point, Point> point_pair = make_pair(*pt_iter, tmp_pt);
			pt_set2.push_back(point_pair);
		}

		double max_dist1 = 0;
		Point raw_point_set1[2], insect_point_set1[2];
		for(vector<pair<Point, Point> >::iterator pair_iter1 = pt_set1.begin();
		    pair_iter1 != pt_set1.end(); pair_iter1++) {
			for(vector<pair<Point, Point> >::iterator pair_iter2 = pt_set1.begin();
			    pair_iter2 != pt_set1.end(); pair_iter2++) {
				double dist1 = getPointDist(pair_iter1->second, pair_iter2->second);
				if(dist1 >= max_dist1) {
					max_dist1 = dist1;
					raw_point_set1[0] = pair_iter1->first;
					raw_point_set1[1] = pair_iter2->first;
					insect_point_set1[0] = pair_iter1->second;
					insect_point_set1[1] = pair_iter2->second;
				}
			}
		}

		double max_dist2 = 0;
		Point raw_point_set2[2], insect_point_set2[2];
		for(vector<pair<Point, Point> >::iterator pair_iter1 = pt_set2.begin();
		    pair_iter1 != pt_set2.end(); pair_iter1++) {
			for(vector<pair<Point, Point> >::iterator pair_iter2 = pt_set2.begin();
			    pair_iter2 != pt_set2.end(); pair_iter2++) {
				double dist2 = getPointDist(pair_iter1->second, pair_iter2->second);
				if(dist2 >= max_dist2) {
					max_dist2 = dist2;
					raw_point_set2[0] = pair_iter1->first;
					raw_point_set2[1] = pair_iter2->first;
					insect_point_set2[0] = pair_iter1->second;
					insect_point_set2[1] = pair_iter2->second;
				}
			}
		}

		double area = max_dist1 * max_dist2;
		/*
		cout << "selected two points in set1 are: " << endl;
		cout << "   origin point1: " << "  x:  " << raw_point_set1[0].x << "  y:  " << raw_point_set1[0].y << endl;
		cout << "   origin point2: " << "  x:  " << raw_point_set1[1].x << "  y:  " << raw_point_set1[1].y << endl;
		cout << "   insect point1: " << "  x:  " << insect_point_set1[0].x << "  y:  " << insect_point_set1[0].y << endl;
		cout << "   insect point2: " << "  x:  " << insect_point_set1[1].x << "  y:  " << insect_point_set1[1].y << endl;

		cout << "selected two points in set2 are: " << endl;
		cout << "   origin point1: " << "  x:  " << raw_point_set2[0].x << "  y:  " << raw_point_set2[0].y << endl;
		cout << "   origin point2: " << "  x:  " << raw_point_set2[1].x << "  y:  " << raw_point_set2[1].y << endl;
		cout << "   insect point1: " << "  x:  " << insect_point_set2[0].x << "  y:  " << insect_point_set2[0].y << endl;
		cout << "   insect point2: " << "  x:  " << insect_point_set2[1].x << "  y:  " << insect_point_set2[1].y << endl;
		*/
		if(area < area_min) {
			area_min = area;
			if(max_dist1 >= max_dist2) {
				select_line = i;
				long_pt[0] = raw_point_set1[0];
				long_pt[1] = raw_point_set1[1];
				short_pt[0] = raw_point_set2[0];
				short_pt[1] = raw_point_set2[1];
				length = max_dist1;
				width = max_dist2;
			} else {
				select_line = (pi / 2) / delta_angle + i;
				long_pt[0] = raw_point_set2[0];
				long_pt[1] = raw_point_set2[1];
				short_pt[0] = raw_point_set1[0];
				short_pt[1] = raw_point_set1[1];
				length = max_dist2;
				width = max_dist1;
			}
		}
	}
//    cout << "The select lines are " << " angle[0]: "  << select_line * delta_angle / pi * 180
//            << " angle[1]: " << (pi / 2 + select_line * delta_angle) / pi * 180 << endl;
	/*
	cout << "The select lines are " << " Lone line angle:  " << select_line * delta_angle / pi * 180
	    << "  Short line angle: " << ((select_line + (int)((pi/2) / delta_angle)) % (int)(pi / delta_angle)) * delta_angle / pi * 180 << endl;

	cout << "long line points: " << " ( " << long_pt[0].x << " , " << long_pt[0].y << " ) "
	        << " ( " << long_pt[1].x << " , " << long_pt[1].y << " ) " << endl;
	cout << "short line points: " << " ( " << short_pt[0].x << " , " << short_pt[0].y << " ) "
	        << " ( " << short_pt[1].x << " , " << short_pt[1].y << " ) " << endl;
	cout << "Area: " << area_min << "  Length:  " << length << "  Width:  " << width << endl;
	*/

	//set the object
	Object obj;
	obj.bbx = getBoxVertex(select_line, long_pt[0], long_pt[1], short_pt[0], short_pt[1]);
	obj.belief = 1;
	obj.length = length;
	obj.width = width;
	obj.anchor_pt = getAnchorPoint_v1(obj.bbx);
	obj.pd = select_line;
	return obj;
}

Point ObjectList::getAnchorPoint(const boundingBox& bbx)
{
	Line ugv_line;
	Point center_pt, anchor_pt;
	//set the center point of bounding box
	center_pt.x = (bbx.vertex[0].x + bbx.vertex[1].x + bbx.vertex[2].x + bbx.vertex[3].x) / 4.0;
	center_pt.y = (bbx.vertex[0].y + bbx.vertex[1].y + bbx.vertex[2].y + bbx.vertex[3].y) / 4.0;
	ugv_line.setLine(ugv_pt, center_pt);
	//set the lines of bounding box
	Line bbx_line[4];
	bbx_line[0].setLine(bbx.vertex[0], bbx.vertex[1]);
	bbx_line[1].setLine(bbx.vertex[1], bbx.vertex[2]);
	bbx_line[2].setLine(bbx.vertex[2], bbx.vertex[3]);
	bbx_line[3].setLine(bbx.vertex[3], bbx.vertex[0]);
	//select the line that has the intersection point
	unsigned int select_line = 0;
	for(unsigned int i = 0; i < 4; i++) {
		if(HaveIntersectPoint(ugv_line, bbx_line[i])) {
			select_line = i;
			break;
		} else {
			cout << "Line " << i << " is not suitable!" << endl;
		}
	}
	cout << "Anchor point: " << select_line << endl;
	anchor_pt.x = (bbx_line[select_line].pt1.x + bbx_line[select_line].pt2.x) / 2.0;
	anchor_pt.y = (bbx_line[select_line].pt1.y + bbx_line[select_line].pt2.y) / 2.0;
	return anchor_pt;
}

bool ObjectList::HaveIntersectPoint(const Line& ugv_line, const Line& box_line)
{
	Point pt = getIntersectPoint(ugv_line, box_line);

	insection_pts.push_back(pt);

	if( ((pt.x >= ugv_line.pt1.x) && (pt.x <= ugv_line.pt2.x))
	    || ((pt.x >= ugv_line.pt2.x) && (pt.x <= ugv_line.pt1.x))) {   //in x
		if( ((pt.y >= ugv_line.pt1.y) && (pt.y <= ugv_line.pt2.y))
		    || ((pt.y >= ugv_line.pt2.y) && (pt.y <= ugv_line.pt1.y))) {   //in y
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}

}

Point ObjectList::getAnchorPoint_v1(const boundingBox& bbx)
{
	Point pt;
	pt.x = (bbx.vertex[0].x + bbx.vertex[1].x + bbx.vertex[2].x + bbx.vertex[3].x) / 4.0;
	pt.y = (bbx.vertex[0].y + bbx.vertex[1].y + bbx.vertex[2].y + bbx.vertex[3].y) / 4.0;
	return pt;
}

Point ObjectList::getIntersectPoint(const Line& line1, const Line& line2)
{
	Point pt;
	double x = (line1.b * line2.c - line2.b * line1.c) / (line1.a * line2.b - line2.a * line1.b);
	double y = (line2.a * line1.c - line1.a * line2.c) / (line1.a * line2.b - line2.a * line1.b);
	pt.setPoint(x, y);
	return pt;
}

Point ObjectList::getIntersectPoint(double a1, double b1, double c1, double a2, double b2, double c2)
{
	Point pt;
	double x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
	double y = (a2 * c1 - a1 * c2) / (a1 * b2 - a2 * b1);
	pt.setPoint(x, y);
	return pt;
}

double ObjectList::getPointDist(const Point& pt1, const Point& pt2)
{
	return sqrt(pow((pt1.x - pt2.x),2) + pow((pt1.y - pt2.y),2));
}

boundingBox ObjectList::getBoxVertex(int long_line, Point long_pt1, Point long_pt2, Point short_pt1, Point short_pt2, int line_pairs)
{
	double la, lb;
	double sa, sb;
	if(long_line == 0) {
		la = 0, lb = 1;
		sa = 1, sb = 0;
	} else if(long_line == line_pairs) {
		la = 1, lb = 0;
		sa = 0, sb = 1;
	} else {
		double angle[2];
		double delta_angle = (pi / 2) / line_pairs;
		angle[0] = long_line * delta_angle, angle[1] = ((long_line + line_pairs) % (line_pairs * 2)) * delta_angle;
		//cout << "Get box vertex: " << "angle 0: " << angle[0] / pi * 180 << " angle 1: " << angle[1] / pi * 180 << endl;
		la = tan(angle[0]), lb = -1;
		sa = tan(angle[1]), sb = -1;
	}
	double lc[2], sc[2];

	//demo: c[0] = - a[0]*head_pt.x - b[0]*head_pt.y;
	lc[0] = - la*short_pt1.x - lb*short_pt1.y;
	lc[1] = - la*short_pt2.x - lb*short_pt2.y;
	sc[0] = - sa*long_pt1.x - sb*long_pt1.y;
	sc[1] = - sa*long_pt2.x - sb*long_pt2.y;

	//cout << "la: " << la << " lb: " << lb << " lc0: " << lc[0] << " lc1: " << lc[1] << endl;
	//cout << "sa: " << sa << " sb: " << sb << " sc0: " << sc[0] << " sc1: " << sc[1] << endl;

	boundingBox bbx;
	bbx.vertex[0] = getIntersectPoint(la, lb, lc[0], sa, sb, sc[0]);
	bbx.vertex[1] = getIntersectPoint(la, lb, lc[0], sa, sb, sc[1]);
	bbx.vertex[2] = getIntersectPoint(la, lb, lc[1], sa, sb, sc[1]);
	bbx.vertex[3] = getIntersectPoint(la, lb, lc[1], sa, sb, sc[0]);
	//sortPoints(bbx);
	return bbx;
}

void ObjectList::setOldObjList()
{

}

void ObjectList::trackObjList()
{
	objs.clear();
	if(!first_record) {
		int corrMat[old_objs.size()][new_objs.size()];
		unsigned int i = 0, j = 0;
		//Here, we use 2 list to save the relationship between ID number and ordinal position
		//The key denotes the ordinal position and the value denotes the ID number
		map<unsigned int, unsigned int> old_list, new_list;
		for(map<unsigned int, Object >::const_iterator old_iter = old_objs.begin();
		    old_iter != old_objs.end(); old_iter++) {
			old_list[i] = old_iter->first;
			i++;
		}
		for(map<unsigned int, Object>::const_iterator new_iter = new_objs.begin();
		    new_iter != new_objs.end(); new_iter++) {
			new_list[j] = new_iter->first;
			j++;
		}

//        cout << "Old list info: " << endl;
//        for(map<unsigned int, unsigned int>::const_iterator iter = old_list.begin();
//                iter != old_list.end(); iter++) {
//            cout << "Key: " << iter->first <<  " Value: " << iter->second << endl;
//        }
//        cout << "New list info: " << endl;
//        for(map<unsigned int, unsigned int>::const_iterator iter = new_list.begin();
//                iter != new_list.end(); iter++) {
//            cout << "Key: " << iter->first <<  " Value: " << iter->second << endl;
//        }

		//compute the correlation matrix between old list and new list
		i = 0;
		for(map<unsigned int, Object >::const_iterator old_iter = old_objs.begin();
		    old_iter != old_objs.end(); old_iter++) {
			j = 0;
			for(map<unsigned int, Object>::const_iterator new_iter = new_objs.begin();
			    new_iter != new_objs.end(); new_iter++) {
				corrMat[i][j] = getCorrlation(old_iter->second, new_iter->second);
				j++;
			}
			i++;
		}

		cout << "The correlaiton matrix is: " << endl;
		for(i = 0; i < old_objs.size(); i++) {
			for(j = 0; j < new_objs.size(); j++) {
				cout << corrMat[i][j] << " ";
			}
			cout << endl;
		}


		//find matched pairs between old list and new list
		matched_pairs1.clear(), matched_pairs2.clear();
		//pair 1: key -- old, value -- new
		//pair 2: key -- new, value -- old
		double corr_max;
		unsigned int old_max = 0, new_max = 0;
		bool noCorr_max = false;

		//if(old_convex_objs.size() > 0 && new_convex_objs.size() > 0) {}

		while(!noCorr_max) {
			//find the max value in correlation matrix and its position
			corr_max = 0;

//            cout << "The correlaiton matrix is(before reduce): " << endl;
//            for(i = 0; i < old_objs.size(); i++) {
//                for(j = 0; j < new_objs.size(); j++) {
//                    cout << corrMat[i][j] << " ";
//                }
//                cout << endl;
//            }

			for(i = 0; i < old_objs.size(); i++) {
				for(j = 0; j < new_objs.size(); j++) {
					if(corrMat[i][j] > corr_max) {
						corr_max = corrMat[i][j];
						old_max = i;
						new_max = j;
					}
				}
			}
			//cout << "old_max: " << old_max << " new_max: " <<new_max << endl;
			//cout << "corr_max: " << corr_max << endl;
			if(corr_max > threshold_corr) {
				//cout << "find the max correlation value!" << endl;
				matched_pairs1[old_max] = new_max;
				matched_pairs2[new_max] = old_max;
				for(j = 0; j < new_objs.size(); j++)
					corrMat[old_max][j] = 0;
				for(i = 0; i < old_objs.size(); i++)
					corrMat[i][new_max] = 0;
			} else {
				//cout << "No max correlation value!" << endl;
				noCorr_max = true;
			}
//          cout << "The correlaiton matrix is(after reduce): " << endl;
//          for(i = 0; i < old_convex_objs.size(); i++) {
//              for(j = 0; j < new_convex_objs.size(); j++) {
//                  cout << corrMat[i][j] << " ";
//              }
//              cout << endl;
//          }
		}

//        cout << "Matched pairs: " << endl;
//        for(map<unsigned int, unsigned int>::const_iterator iter = matched_pairs1.begin();
//                iter != matched_pairs1.end(); iter++) {
//            cout << iter->first << " " << iter->second << endl;
//       }

		objs.clear();
		unsigned int old_cnt = 0, new_cnt = 0;

		//find the object in old list
		for(i = 0; i < old_objs.size(); i++) {
			if(matched_pairs1.count(i) != 0) {
				//old_list.find(i)->second denotes ID number of object in old convex list
				//matched_pairs1.find(i)->second denotes the corresponding ordinal position
				//new_list.find(matched_pairs1.find(i)->second)->second denotes ID number of object in new convex list
				Object old_obj = old_objs.find(old_list.find(i)->second)->second;
				Object new_obj = new_objs.find((new_list.find(matched_pairs1.find(i)->second)->second))->second;
				Object obj = updateMatchedObject(old_obj, new_obj);
				objs[old_cnt] = obj;
				old_cnt++;
			} else {
				Object obj = updateOldListObject(old_objs.find(old_list.find(i)->second)->second);
				if(obj.belief > min_belief) {
					objs[old_cnt] = obj;
					old_cnt++;
				}
			}
		}
		//find the object in new list
		for(j = 0; j < new_objs.size(); j++) {
			if(matched_pairs2.count(j) == 0) {
				new_cnt++;
				Object obj = updateNewListObject(new_objs.find(new_list.find(j)->second)->second);
				objs[old_cnt + new_cnt] = obj;
			}
		}

		//print info if you want
		/*
		cout << "--------------------------------------------------------------" << endl;
		cout << "Objects in new list: " << endl;
		for(map<unsigned int, Object>::const_iterator map_iter = new_objs.begin();
		        map_iter != new_objs.end(); map_iter++) {
		    cout << "   Object " << map_iter->first << endl;
		    cout << "       Position: " << " ( " << map_iter->second.anchor_pt.x << " , " << map_iter->second.anchor_pt.y << " ) " << endl;
		    cout << "       PD:       " << map_iter->second.pd     << " Length:   " << map_iter->second.length << " Width: " << map_iter->second.width << endl;
			cout << "Belief:   " << map_iter->second.belief << " Type: "  << (int)map_iter->second.type << endl;
			cout << "            Vertex[0]: " << "( " << map_iter->second.bbx.vertex[0].x << " , " << map_iter->second.bbx.vertex[0].y << " ) " << endl;
		    cout << "            Vertex[1]: " << "( " << map_iter->second.bbx.vertex[1].x << " , " << map_iter->second.bbx.vertex[1].y << " ) " << endl;
		    cout << "            Vertex[2]: " << "( " << map_iter->second.bbx.vertex[2].x << " , " << map_iter->second.bbx.vertex[2].y << " ) " << endl;
		    cout << "            Vertex[3]: " << "( " << map_iter->second.bbx.vertex[3].x << " , " << map_iter->second.bbx.vertex[3].y << " ) " << endl;
		}
		*/
		/*
		cout << "--------------------------------------------------------------" << endl;
		cout << "Objects in old list: " << endl;
		for(map<unsigned int, Object>::const_iterator map_iter = new_objs.begin();
		        map_iter != new_objs.end(); map_iter++) {
		    cout << "   Object " << map_iter->first << endl;
		    cout << "       Position: " << " ( " << map_iter->second.anchor_pt.x << " , " << map_iter->second.anchor_pt.y << " ) " << endl;
		    cout << "       PD:       " << map_iter->second.pd     << " Length:   " << map_iter->second.length << " Width: " << map_iter->second.width << endl;
		    cout << "       Belief:   " << map_iter->second.belief << " Type: "  << map_iter->second.type << endl;
		    cout << "            Vertex[0]: " << "( " << map_iter->second.bbx.vertex[0].x << " , " << map_iter->second.bbx.vertex[0].y << " ) " << endl;
		    cout << "            Vertex[1]: " << "( " << map_iter->second.bbx.vertex[1].x << " , " << map_iter->second.bbx.vertex[1].y << " ) " << endl;
		    cout << "            Vertex[2]: " << "( " << map_iter->second.bbx.vertex[2].x << " , " << map_iter->second.bbx.vertex[2].y << " ) " << endl;
		    cout << "            Vertex[3]: " << "( " << map_iter->second.bbx.vertex[3].x << " , " << map_iter->second.bbx.vertex[3].y << " ) " << endl;
		}
		*/
		/*
		cout << "--------------------------------------------------------------" << endl;
		cout << "Objects in final list: " << endl;
		for(map<unsigned int, Object>::const_iterator map_iter = objs.begin();
		        map_iter != objs.end(); map_iter++) {
		    cout << "Object " << map_iter->first << endl;
		    cout << "Position: " << " ( " << map_iter->second.anchor_pt.x << " , " << map_iter->second.anchor_pt.y << " ) " << endl;
		    cout << "PD:       " << map_iter->second.pd     << " Length:   " << map_iter->second.length << " Width: " << map_iter->second.width << endl;
		    cout << "Belief:   " << map_iter->second.belief << " Type: "  << (int)map_iter->second.type
				<< " Shift: ( " << map_iter->second.shift.x << " , " << map_iter->second.shift.y << " ) " << endl;
		    cout << "    Vertex[0]: " << "( " << map_iter->second.bbx.vertex[0].x << " , " << map_iter->second.bbx.vertex[0].y << " ) " << endl;
		    cout << "    Vertex[1]: " << "( " << map_iter->second.bbx.vertex[1].x << " , " << map_iter->second.bbx.vertex[1].y << " ) " << endl;
		    cout << "    Vertex[2]: " << "( " << map_iter->second.bbx.vertex[2].x << " , " << map_iter->second.bbx.vertex[2].y << " ) " << endl;
		    cout << "    Vertex[3]: " << "( " << map_iter->second.bbx.vertex[3].x << " , " << map_iter->second.bbx.vertex[3].y << " ) " << endl;
		}
		*/
		matched_pairs1.clear();
		matched_pairs2.clear();
		old_objs.clear();
		old_objs = objs;
	} else {
		first_record = false;
		objs = new_objs;
		old_objs.clear();
		old_objs = objs;
	}
}

int ObjectList::getCorrlation(const Object& old_obj, const Object& new_obj, float location_weight, float shape_weight)
{
//    int c = 0;
//    double c1 = pow(((old_obj.anchor_pt.x - new_obj.anchor_pt.x) * 5), 2) + pow(((old_obj.anchor_pt.y - new_obj.anchor_pt.y) * 5), 2);
//    double c2 = pow((old_obj.length - new_obj.length), 2) + pow((old_obj.width - new_obj.width), 2);
//    double c11 = 0, c22 = 0;
//
//    if(c1 <= 200)
//        c11 = (200 - c1) / 10;
//
//    //cout << "c1: " << c11 << endl;
//    if(c2 <= 100)
//        c22 = (100 - c2) / 10;
//
//    //cout << "c2: " << c22 << endl;
//
//
//    c = c11 + c22;
//    //cout << "c: " << c << endl;
//    return c;

	int location_corr = getLocationCorrlation(old_obj, new_obj);
	int shape_corr = getShapeCorrlation(old_obj, new_obj);
	int corr = location_corr * location_weight + shape_corr * shape_weight;

	//cout << "location value: " << location_corr << " shape_corr: " << shape_corr << endl;

	return corr;
}

int ObjectList::getShapeCorrlation(const Object& old_obj, const Object& new_obj)
{
	float diff_length = fabsf(old_obj.length - new_obj.length);
	float diff_width = fabsf(old_obj.width - new_obj.width);
	float length_value = (-100.0 / 3.0) * diff_length + 100.0;
	float width_value = (-100.0 / 2.0) * diff_width + 100.0;

	unsigned int diff_pd = abs(old_obj.pd - new_obj.pd);
	float pd_value = (-100.0 / 11.0) * diff_pd + 100.0;

	if(length_value < 0)
		length_value = 0;
	if(width_value < 0)
		width_value = 0;
	if(pd_value < 0)
		pd_value = 0;

	float value = 0.4 * length_value + 0.3 * width_value + 0.3 * width_value;

	//cout << "length value: " << length_value << " width value: " << width_value << " pd_value: " << pd_value << endl;
	return value;

}

int ObjectList::getLocationCorrlation(const Object& old_obj, const Object& new_obj)
{
	float diff_x = fabsf(old_obj.anchor_pt.x - new_obj.anchor_pt.x);
	float diff_y = fabsf(old_obj.anchor_pt.y - new_obj.anchor_pt.y);
	float diff_dist = sqrtf(pow(diff_x, 2)+ pow(diff_y, 2));

	float value = (-100.0 / 3.0) * diff_dist + 100.0;
	if(value >= 0)
		return value;
	else
		return 0;
}

Object ObjectList::updateMatchedObject(const Object& old_obj, const Object& new_obj)
{
	Object obj = new_obj;
	obj.type = old_obj.type;
	obj.belief = old_obj.belief;
	obj.belief++;
	if(obj.belief >= 20)
		obj.belief = 20;

	obj.obj_lcs = old_obj.obj_lcs;
	obj.updateObjInfoDeque(new_obj.anchor_pt, tv);
	obj.setObjInfo();

	if(obj.belief >= 12) {
		if(obj.type == 1) {		//static
			if(getObjectType(true, obj.sum_shift) == 2) {
				obj.type = 2;
			} else {
				obj.type = 1;
			}
		} else if(obj.type == 2) {	//dynamic
			obj.type = 2;
		} else {					//unknown
			obj.type = getObjectType(true, obj.sum_shift);
		}
	} else {
		obj.type = 0;
	}
	return obj;
}

unsigned char ObjectList::getObjectType_v1(bool BaseOnShift, const Point& shift,
        double dynamic_shift, double static_shift,
        double dynamic_velocity, double static_velocity)
{
	double d = sqrt(pow(shift.x, 2) + pow(shift.y, 2));
	if(BaseOnShift) {
		unsigned char ch;
		if(d >= dynamic_shift)
			ch = 2;
		else if(d <= static_shift)
			ch = 1;
		else
			ch = 0;
		return ch;
	} else {
		unsigned char ch;
		if(d >= dynamic_velocity)
			ch = 2;
		else if(d <= static_velocity)
			ch = 1;
		else
			ch = 0;
		return ch;
	}
}

unsigned char ObjectList::getObjectType(bool BaseOnShift, const Point& shift,
                                        double dynamic_shift, double static_shift,
                                        double dynamic_velocity, double static_velocity)
{
	double d = sqrt(pow(shift.x, 2) + pow(shift.y, 2));
	if(BaseOnShift) {
		unsigned char ch;
		if(d >= dynamic_shift)
			ch = 2;
		else if(d <= static_shift)
			ch = 1;
		else
			ch = 0;
		return ch;
	} else {
		unsigned char ch;
		if(d >= dynamic_velocity)
			ch = 2;
		else if(d <= static_velocity)
			ch = 1;
		else
			ch = 0;
		return ch;
	}
}

Object ObjectList::updateOldListObject(const Object& old_obj)
{
	Object obj = old_obj;
	obj.belief -- ;
	if(obj.belief <= 0)
		obj.belief = 0;
	return obj;
}

Object ObjectList::updateNewListObject(const Object& new_obj)
{
	Object obj = new_obj;
	obj.belief = 1;
	return obj;
}

void ObjectList::transOccupyGrids(const vector<cluster_pos_t>& global_convex_pos)
{
	local_convex_pos.clear();
	for(vector<cluster_pos_t>::const_iterator pos_iter = global_convex_pos.begin();
	    pos_iter != global_convex_pos.end(); pos_iter++) {
		cluster_pos_t lp = getLocalPos(*pos_iter);
		local_convex_pos.push_back(lp);
	}
}

cluster_pos_t ObjectList::getLocalPos(cluster_pos_t global_pos)
{
	cluster_pos_t lp;
	lp.row = global_pos.row - ugv_pos.row;
	lp.col = global_pos.col - ugv_pos.col;
	return lp;
}

void ObjectList::transObjects(const map<unsigned int, Object>& objs)
{
	cout << "Local objects in final list: " << endl;
	double closest_dist = 10000.0;
	HaveClosestObj = false;
	if(objs.size() > 0) {
		map<unsigned int, Object>::const_iterator beg_iter = objs.begin();
		closest_obj.type     =   beg_iter->second.type;
		closest_obj.belief   =   beg_iter->second.belief;
		//closest_obj.width    =   beg_iter->second.width;
		//closest_obj.length   =   beg_iter->second.length;
		//closest_obj.pd       =   beg_iter->second.pd;
		closest_obj.velocity =   beg_iter->second.velocity;
		//closest_obj.sum_shift    =   beg_iter->second.sum_shift;
		closest_obj.obj_lcs.clear();
		closest_obj.obj_shifts.clear();
		closest_obj.anchor_pt = getLocalPoint(beg_iter->second.anchor_pt);
		closest_obj.bbx = getLocalBoundingBox(beg_iter->second.bbx);

		closest_obj.ugv_anchor_pt = getUGVLocalPoint(closest_obj.anchor_pt);
		closest_obj.velocity = getUGVLocalPoint(closest_obj.velocity);

		double dist = sqrt(pow(closest_obj.ugv_anchor_pt.x, 2) + pow(closest_obj.ugv_anchor_pt.y, 2));
		if(closest_obj.belief >= 8 && dist < closest_dist) {
			HaveClosestObj = true;
			closest_dist = dist;
		}
	}


//    for(map<unsigned int, Object>::const_iterator map_iter = objs.begin();
//            map_iter != objs.end(); map_iter++) {
//        cout << "Object " << map_iter->first << endl;
//        cout << "Position: " << " ( " << map_iter->second.anchor_pt.x << " , " << map_iter->second.anchor_pt.y << " ) " << endl;
//        cout << "Length:   " << map_iter->second.length << " Width: " << map_iter->second.width << endl;
//        cout << "Belief:   " << map_iter->second.belief << " Type: "  << (int)map_iter->second.type << endl;
//        cout << "    Vertex[0]: " << "( " << map_iter->second.bbx.vertex[0].x << " , " << map_iter->second.bbx.vertex[0].y << " ) " << endl;
//        cout << "    Vertex[1]: " << "( " << map_iter->second.bbx.vertex[1].x << " , " << map_iter->second.bbx.vertex[1].y << " ) " << endl;
//        cout << "    Vertex[2]: " << "( " << map_iter->second.bbx.vertex[2].x << " , " << map_iter->second.bbx.vertex[2].y << " ) " << endl;
//        cout << "    Vertex[3]: " << "( " << map_iter->second.bbx.vertex[3].x << " , " << map_iter->second.bbx.vertex[3].y << " ) " << endl;
//    }

	local_objs.clear();
	for(map<unsigned int, Object>::const_iterator map_iter = objs.begin();
	    map_iter != objs.end(); map_iter++) {
		Object lobj;
		lobj.type     =   map_iter->second.type;
		lobj.belief   =   map_iter->second.belief;
		lobj.width    =   map_iter->second.width;
		lobj.length   =   map_iter->second.length;
		lobj.pd       =   map_iter->second.pd;
		lobj.velocity =   map_iter->second.velocity;
		lobj.sum_shift    =   map_iter->second.sum_shift;
		lobj.obj_lcs.clear();
		lobj.obj_shifts.clear();
		lobj.anchor_pt = getLocalPoint(map_iter->second.anchor_pt);
		lobj.bbx = getLocalBoundingBox(map_iter->second.bbx);
		lobj.ugv_anchor_pt = getUGVLocalPoint(lobj.anchor_pt);

		double dist = sqrt(pow(closest_obj.ugv_anchor_pt.x, 2) + pow(closest_obj.ugv_anchor_pt.y, 2));
		if(lobj.belief >= 8 && dist < closest_dist) {
			HaveClosestObj = true;
			closest_dist = dist;

			closest_obj.type     =   map_iter->second.type;
			closest_obj.belief   =   map_iter->second.belief;

			closest_obj.velocity =   map_iter->second.velocity;

			closest_obj.obj_lcs.clear();
			closest_obj.obj_shifts.clear();
			closest_obj.anchor_pt = lobj.anchor_pt;
			closest_obj.bbx = lobj.bbx;

			closest_obj.ugv_anchor_pt = getUGVLocalPoint(lobj.anchor_pt);
			closest_obj.velocity = getUGVLocalPoint(lobj.velocity);
		}

		//used for test velocity
//        lobj.velocity.x = 1.0;
//        lobj.velocity.y = 1.0;

		local_objs[map_iter->first] = lobj;


	}

	for(map<unsigned int, Object>::iterator map_iter = local_objs.begin();
	    map_iter != local_objs.end(); map_iter++) {
		cout << "Object " << map_iter->first << endl;
		cout << "Position: " << " ( " << map_iter->second.ugv_anchor_pt.x << " , " << map_iter->second.ugv_anchor_pt.y << " ) " << endl;
		cout << "PD:     " << map_iter->second.pd     << "   Length: " << map_iter->second.length << "    Width: " << map_iter->second.width << endl;
		cout << "Belief: " << map_iter->second.belief << "   TYPE: "  << (int)map_iter->second.type << endl;
		cout << "Sum Shift: " << sqrt(pow(map_iter->second.sum_shift.x, 2) + pow(map_iter->second.sum_shift.y, 2)) << "  ( " << map_iter->second.sum_shift.x  << " , " << map_iter->second.sum_shift.y <<  " ) " << endl;
		cout << "Velocity:  " << sqrt(pow(map_iter->second.velocity.x, 2)  + pow(map_iter->second.velocity.y, 2))  << "  ( " << map_iter->second.velocity.x   << " , " << map_iter->second.velocity.y  << " ) "
		     << " RV:   ( " << getUGVLocalPoint(map_iter->second.velocity).x   << " , " << getUGVLocalPoint(map_iter->second.velocity).y  << " ) "<<  endl;

		cout << "    Vertex[0]: " << "( " << map_iter->second.bbx.vertex[0].x << " , " << map_iter->second.bbx.vertex[0].y << " ) " << endl;
		cout << "    Vertex[1]: " << "( " << map_iter->second.bbx.vertex[1].x << " , " << map_iter->second.bbx.vertex[1].y << " ) " << endl;
		cout << "    Vertex[2]: " << "( " << map_iter->second.bbx.vertex[2].x << " , " << map_iter->second.bbx.vertex[2].y << " ) " << endl;
		cout << "    Vertex[3]: " << "( " << map_iter->second.bbx.vertex[3].x << " , " << map_iter->second.bbx.vertex[3].y << " ) " << endl;

//		if(map_iter->second.anchor_pt.x <= 80 && map_iter->second.anchor_pt.x >= -80
//				&& map_iter->second.anchor_pt.y <= 80 && map_iter->second.anchor_pt.y >= -80) {
//			cout << "               Available object!" << endl;
//		} else {
//			cout << "               Unavailable object!" << endl;
//			//objs.erase(map_iter->first);
//		}

		if(db.InDetectBox(map_iter->second)) {
			cout << "  Available object: In";
			if(ver_db.InDetectBox(map_iter->second)) {
				cout << "  Vertical Detect Box ";
			} else if(horLeft_db.InDetectBox(map_iter->second)) {
				cout << " Left Horizontal Detect Box ";
				Point rv;
				rv = getUGVLocalPoint(map_iter->second.velocity);
				if(rv.x > 0.5)
					cout << "  !!!!!!!Dangerous!!!!!!!" << endl;

			} else if(horRight_db.InDetectBox(map_iter->second)) {
				cout << " Right Horizontal Detect Box ";
				Point rv;
				rv = getUGVLocalPoint(map_iter->second.velocity);
				if(rv.x < -0.5)
					cout << "  !!!!!!!Dangerous!!!!!!!" << endl;
			} else if(left_db.InDetectBox(map_iter->second)) {
				cout << "  Left Detect Box ";
			} else if(right_db.InDetectBox(map_iter->second)) {
				cout << " Right Detect Box ";
			}
			cout << endl;
		} else {
			cout << "  Unavailable object!" << endl;
		}

	}
}

Point ObjectList::getLocalPoint(Point global_pt)
{
	Point local_pt;
	local_pt.setPoint(global_pt.x - ugv_pt.x, global_pt.y - ugv_pt.y);
	return local_pt;
}

Point ObjectList::getUGVLocalPoint(Point local_pt)
{
	Point ugv_local_pt;
	float x = cos(current_pose.eulr) * local_pt.x - sin(current_pose.eulr) * local_pt.y;
	float y = sin(current_pose.eulr) * local_pt.x + cos(current_pose.eulr) * local_pt.y;
	ugv_local_pt.x = x;
	ugv_local_pt.y = y;
	return ugv_local_pt;
}

boundingBox ObjectList::getLocalBoundingBox(const boundingBox& global_bbx)
{
	boundingBox local_bbx;
	local_bbx.vertex[0] = getLocalPoint(global_bbx.vertex[0]);
	local_bbx.vertex[1] = getLocalPoint(global_bbx.vertex[1]);
	local_bbx.vertex[2] = getLocalPoint(global_bbx.vertex[2]);
	local_bbx.vertex[3] = getLocalPoint(global_bbx.vertex[3]);
	return local_bbx;
}

void ObjectList::getTestOccupyGrids()
{
	gettimeofday(&tv, NULL);
	convex_pos.clear();
	unsigned int pos_cnt = 0;
	for(vector<Point>::const_iterator pt_iter = click_pts.begin();
	    pt_iter != click_pts.end(); pt_iter++) {
		cluster_pos_t tpos;
		tpos.id = pos_cnt;
		tpos.row = (int)(pt_iter->y / 0.2);
		tpos.col = (int)(pt_iter->x / 0.2);
		convex_pos.push_back(tpos);
		pos_cnt++;
	}

}



//the following functions are old functions
bool ObjectList::sharpenClusters()
{
	cluster_pts.clear();
	objs.clear();

	unsigned int cluster_cnt = 0;
	for(map<unsigned int, vector<cluster_pos_t> >::iterator map_iter = clusters.begin();
	    map_iter != clusters.end(); map_iter++) {
		for(vector<cluster_pos_t>::iterator pos_iter = map_iter->second.begin();
		    pos_iter != map_iter->second.end(); pos_iter++) {
			Point tmp_pt;
//            tmp_pt.x = getx(pos_iter->row, pos_iter->col);
//            tmp_pt.y = gety(pos_iter->row, pos_iter->col);
			tmp_pt.x = pos_iter->col * 0.2;
			tmp_pt.y = pos_iter->row * 0.2;
			cluster_pts[cluster_cnt].push_back(tmp_pt);
		}
		cluster_cnt++;
	}

	for(map<unsigned int, vector<Point> >::iterator map_iter = cluster_pts.begin();
	    map_iter != cluster_pts.end(); map_iter++) {
		cout << "cluster " << map_iter->first << " contains points: " << endl;
		for(vector<Point>::iterator pt_iter = map_iter->second.begin();
		    pt_iter != map_iter->second.end(); pt_iter++) {
			cout << "       " << " x: "  << pt_iter->x   << "   y: " << pt_iter->y << endl;
		}
	}

	//generate 6 lines for test
	for(map<unsigned int, vector<Point> >::iterator map_iter = cluster_pts.begin();
	    map_iter != cluster_pts.end(); map_iter++) {
		//objs[map_iter->first] = getBoundingBox(map_iter->second);
	}

//    for(map<unsigned int, object_t>::const_iterator map_iter = objs.begin();
//            map_iter != objs.end(); map_iter++) {
//        cout << "object " << map_iter->first << endl;
//        cout << "       " << "vertex0: ( " << map_iter->second.bbx.vertex[0].x << " , " << map_iter->second.bbx.vertex[0].y << " ) " << endl;
//        cout << "       " << "vertex1: ( " << map_iter->second.bbx.vertex[1].x << " , " << map_iter->second.bbx.vertex[1].y << " ) " << endl;
//        cout << "       " << "vertex2: ( " << map_iter->second.bbx.vertex[2].x << " , " << map_iter->second.bbx.vertex[2].y << " ) " << endl;
//        cout << "       " << "vertex3: ( " << map_iter->second.bbx.vertex[3].x << " , " << map_iter->second.bbx.vertex[3].y << " ) " << endl;
//    }
	return true;
}

rm_object_t ObjectList::getBoundingBox(const vector<Point>& pts, int line_pairs)
{
	//int line_pairs = 6;
	double delta_angle = (pi / 2) / line_pairs;     //15
	//cout << "delta angle: " << delta_angle / pi * 180 << endl;

	double area_min = 10000.0;
	int select_line = 0;

	Point long_pt[2], short_pt[2];
	//double a_max, b_max, c_max;
	//a_max = b_max = c_max = 0;

	//select the line pair
	for(int i = 0; i < line_pairs; i++) {
		double angle[2];
		//double tanValue[2];
		double a[2], b[2], c[2];
		angle[0] = i * delta_angle;
		angle[1] = angle[0] + pi / 2;
		cout << "--------------------------------------------------------" << endl;
		cout << "angle 0: " <<  angle[0] / pi * 180 << " angle 1: " <<  angle[1] / pi * 180 << endl;
		if(i == 0) {
			a[0] = 0;
			b[0] = 1;
			a[1] = 1;
			b[1] = 0;
		} else {
			a[0] = tan(angle[0]);
			b[0] = -1;
			a[1] = tan(angle[1]);
			b[1] = -1;
		}
		Point head_pt = *(pts.begin());

		c[0] = - a[0]*head_pt.x - b[0]*head_pt.y;

		//Here, pt_set1 and pt_set2 are point sets. Their elements are intersection points
		//First element is the origin point. Second element is the intersection point
		//vector<Point> pt_set1;
		vector<pair<Point, Point> > pt_set1;
		for(vector<Point>::const_iterator pt_iter = pts.begin();
		    pt_iter != pts.end(); pt_iter++) {
			double c1 = - a[1] * pt_iter->x - b[1] * pt_iter->y;
			Point tmp_pt = getIntersectPoint(a[0], b[0], c[0], a[1], b[1], c1);
			//pt_set1.push_back(tmp_pt);
			pair<Point, Point> point_pair = make_pair(*pt_iter, tmp_pt);
			pt_set1.push_back(point_pair);
		}

		c[1] = - a[1] * head_pt.x - b[1] * head_pt.y;
		//vector<Point> pt_set2;
		vector<pair<Point, Point> > pt_set2;
		for(vector<Point>::const_iterator pt_iter = pts.begin();
		    pt_iter != pts.end(); pt_iter++) {
			double c0 = - a[0] * pt_iter->x - b[0] * pt_iter->y;
			Point tmp_pt = getIntersectPoint(a[0], b[0], c0, a[1], b[1], c[1]);
			//pt_set2.push_back(tmp_pt);
			pair<Point, Point> point_pair = make_pair(*pt_iter, tmp_pt);
			pt_set2.push_back(point_pair);
		}

		//scan the points set, find the biggest distance
		//the iterators including pt_iter_set1, pt_iter_set2 are pointing to intersection points

//        double max_dist1 = 0;
//        vector<Point>::iterator pt_iter_set1[2];
//        pt_iter_set1[0] = pt_iter_set1[1] = pt_set1.begin();
//        for(vector<Point>::iterator pt_set_iter1 = pt_set1.begin();
//                pt_set_iter1 != pt_set1.end(); pt_set_iter1++) {
//            for(vector<Point>::iterator pt_set_iter2 = pt_set1.begin();
//                    pt_set_iter2 != pt_set1.end(); pt_set_iter2++) {
//                double dist1 = getPointDist(*pt_set_iter1, *pt_set_iter2);
//                if(dist1 > max_dist1) {
//                    pt_iter_set1[0] = pt_set_iter1;
//                    pt_iter_set1[1] = pt_set_iter2;
//                    max_dist1 = dist1;
//                }
//            }
//        }

//        double max_dist2 = 0;
//        vector<Point>::iterator pt_iter_set2[2];
//        pt_iter_set2[0] = pt_iter_set2[1] = pt_set2.begin();
//        for(vector<Point>::iterator pt_set_iter1 = pt_set2.begin();
//                pt_set_iter1 != pt_set2.end(); pt_set_iter1++) {
//            for(vector<Point>::iterator pt_set_iter2 = pt_set2.begin();
//                    pt_set_iter2 != pt_set2.end(); pt_set_iter2++) {
//                double dist2 = getPointDist(*pt_set_iter1, *pt_set_iter2);
//                if(dist2 > max_dist2) {
//                    pt_iter_set2[0] = pt_set_iter1;
//                    pt_iter_set2[1] = pt_set_iter2;
//                    max_dist2 = dist2;
//                }
//            }
//        }

		double max_dist1 = 0;
		Point raw_point_set1[2], insect_point_set1[2];
		for(vector<pair<Point, Point> >::iterator pair_iter1 = pt_set1.begin();
		    pair_iter1 != pt_set1.end(); pair_iter1++) {
			for(vector<pair<Point, Point> >::iterator pair_iter2 = pt_set1.begin();
			    pair_iter2 != pt_set1.end(); pair_iter2++) {
				double dist1 = getPointDist(pair_iter1->second, pair_iter2->second);
				if(dist1 >= max_dist1) {
					max_dist1 = dist1;
					raw_point_set1[0] = pair_iter1->first;
					raw_point_set1[1] = pair_iter2->first;
					insect_point_set1[0] = pair_iter1->second;
					insect_point_set1[1] = pair_iter2->second;
				}
			}
		}

		double max_dist2 = 0;
		Point raw_point_set2[2], insect_point_set2[2];
		for(vector<pair<Point, Point> >::iterator pair_iter1 = pt_set2.begin();
		    pair_iter1 != pt_set2.end(); pair_iter1++) {
			for(vector<pair<Point, Point> >::iterator pair_iter2 = pt_set2.begin();
			    pair_iter2 != pt_set2.end(); pair_iter2++) {
				double dist2 = getPointDist(pair_iter1->second, pair_iter2->second);
				if(dist2 >= max_dist2) {
					max_dist2 = dist2;
					raw_point_set2[0] = pair_iter1->first;
					raw_point_set2[1] = pair_iter2->first;
					insect_point_set2[0] = pair_iter1->second;
					insect_point_set2[1] = pair_iter2->second;
				}
			}
		}

		double area = max_dist1 * max_dist2;

		//print the points in set 1
//        cout << "Points in set 1: " << endl;
//        for(vector<Point>::const_iterator pt_iter = pt_set1.begin();
//                pt_iter != pt_set1.end(); pt_iter++) {
//            cout << "   " << " x: " << pt_iter->x << "  y: " << pt_iter->y << endl;
//        }
//        //print the points in set 1
//        cout << "Points in set 2: " << endl;
//        for(vector<Point>::const_iterator pt_iter = pt_set2.begin();
//                pt_iter != pt_set2.end(); pt_iter++) {
//            cout << "   " << " x: " << pt_iter->x << "  y: " << pt_iter->y << endl;
//        }

//        cout << "selected two points int set 1 are: " << endl;
//        cout << "   point 1: " << "  x:  " << pt_iter_set1[0]->x << "  y:  " << pt_iter_set1[0]->y << endl;
//        cout << "   point 2: " << "  x:  " << pt_iter_set1[1]->x << "  y:  " << pt_iter_set1[1]->y << endl;
//
//        cout << "selected two points int set 2 are: " << endl;
//        cout << "   point 1: " << "  x:  " << pt_iter_set2[0]->x << "  y:  " << pt_iter_set2[0]->y << endl;
//        cout << "   point 2: " << "  x:  " << pt_iter_set2[1]->x << "  y:  " << pt_iter_set2[1]->y << endl;
//
//        cout << "max dist1: " << max_dist1 << endl;
//        cout << "max dist2: " << max_dist2 << endl;
//        cout << "current area: " << area << endl;

		cout << "selected two points in set1 are: " << endl;
		cout << "   origin point1: " << "  x:  " << raw_point_set1[0].x << "  y:  " << raw_point_set1[0].y << endl;
		cout << "   origin point2: " << "  x:  " << raw_point_set1[1].x << "  y:  " << raw_point_set1[1].y << endl;
		cout << "   insect point1: " << "  x:  " << insect_point_set1[0].x << "  y:  " << insect_point_set1[0].y << endl;
		cout << "   insect point2: " << "  x:  " << insect_point_set1[1].x << "  y:  " << insect_point_set1[1].y << endl;

		cout << "selected two points in set2 are: " << endl;
		cout << "   origin point1: " << "  x:  " << raw_point_set2[0].x << "  y:  " << raw_point_set2[0].y << endl;
		cout << "   origin point2: " << "  x:  " << raw_point_set2[1].x << "  y:  " << raw_point_set2[1].y << endl;
		cout << "   insect point1: " << "  x:  " << insect_point_set2[0].x << "  y:  " << insect_point_set2[0].y << endl;
		cout << "   insect point2: " << "  x:  " << insect_point_set2[1].x << "  y:  " << insect_point_set2[1].y << endl;

		if(area < area_min) {
			area_min = area;
			if(max_dist1 >= max_dist2) {
				select_line = i;
				long_pt[0] = raw_point_set1[0];
				long_pt[1] = raw_point_set1[1];
				short_pt[0] = raw_point_set2[0];
				short_pt[1] = raw_point_set2[1];
			} else {
				select_line = (pi / 2) / delta_angle + i;
				long_pt[0] = raw_point_set2[0];
				long_pt[1] = raw_point_set2[1];
				short_pt[0] = raw_point_set1[0];
				short_pt[1] = raw_point_set1[1];
			}
		}
	}
//    cout << "The select lines are " << " angle[0]: "  << select_line * delta_angle / pi * 180
//            << " angle[1]: " << (pi / 2 + select_line * delta_angle) / pi * 180 << endl;

	cout << "The select lines are " << " Lone line angle:  " << select_line * delta_angle / pi * 180
	     << "  Short line angle: " << ((select_line + (int)((pi/2) / delta_angle)) % (int)(pi / delta_angle)) * delta_angle / pi * 180 << endl;

	cout << "long line points: " << " ( " << long_pt[0].x << " , " << long_pt[0].y << " ) "
	     << " ( " << long_pt[1].x << " , " << long_pt[1].y << " ) " << endl;
	cout << "short line points: " << " ( " << short_pt[0].x << " , " << short_pt[0].y << " ) "
	     << " ( " << short_pt[1].x << " , " << short_pt[1].y << " ) " << endl;

	rm_object_t tmp_obj;
	tmp_obj.bbx = getBoxVertex(select_line, long_pt[0], long_pt[1], short_pt[0], short_pt[1]);
	return tmp_obj;
}

void ObjectList::sortPoints(boundingBox& bbx)
{
//    Point v[4];
//    v[0] = bbx.vertex[0];
//    v[1] = bbx.vertex[1];
//    v[2] = bbx.vertex[2];
//    v[3] = bbx.vertex[3];
//
//    double average_x = (v[0].x + v[1].x + v[2].x + v[3].x) / 4.0;
//    double average_y = (v[0].y + v[1].y + v[2].y + v[3].y) / 4.0;
//
//    //int l1, l2, r1, r2;
//
//    int v0, v1, v2, v3;

//    cout << "v0:  " << v0 << "  v1:  " << v1 << "  v2:  " << v2 << "  v3:  " << v3 << endl;
//    bbx.vertex[0] = v[v0];
//    bbx.vertex[1] = v[v1];
//    bbx.vertex[2] = v[v2];
//    bbx.vertex[3] = v[v3];

}

//Object ObjectList::cv_setNewObject(const vector<Point>& pts)
//{
//    vector<cv::Point2d> cv_pts;
//    for(vector<Point>::const_iterator pt_iter = pts.begin();
//            pt_iter != pts.end(); pt_iter++) {
//        cv::Point2d tmp_pt(pt_iter->x, pt_iter->y);
//        cv_pts.push_back(tmp_pt);
//    }
//
//    cv::RotatedRect cv_rrt = cv::minAreaRect(cv_pts);
//
//    set the object
//    Object obj;
//    obj.bbx = getBoxVertex(select_line, long_pt[0], long_pt[1], short_pt[0], short_pt[1]);
//    obj.belief = 1;
//    obj.length = length;
//    obj.width = width;
//    obj.anchor_pt = getAnchorPoint(obj.bbx);
//    obj.pd = select_line;
//}

void ObjectList::setPathFlag()
{
	bool flag;

	if(HaveClosestObj && closest_obj.belief >= 8) {
		flag = true;
	} else {
		flag = false;
	}
	//set the flat to shared memory
	//module::RecoData_t m_objectList;

	module::MarkerData_t m_markerData;
	m_markerData.type = module::MarkerData_t::MARKER_DP;
	m_markerData.value.v_dp.mark = flag;
	module::shm::SHARED_OBJECTS.SetMarker(m_markerData);
}

void ObjectList::setTrackObstacle()
{
	bool flag;
	//double dist, veloy;
	if(HaveClosestObj && closest_obj.belief >= 8) {
		flag = true;
		track_obs.type = closest_obj.type;
		track_obs.anchor_pt.x = closest_obj.ugv_anchor_pt.x;
		track_obs.anchor_pt.y = closest_obj.ugv_anchor_pt.y;
		track_obs.velocity.x = closest_obj.velocity.x;
		track_obs.velocity.y = closest_obj.velocity.y;

	} else {
		flag = false;
	}
	//set the track obs to shared memory
	module::MarkerData_t m_markerData;
	m_markerData.type = module::MarkerData_t::MARKER_OBJC;
	if(flag) {
		if(track_obs.type == 0) {
			m_markerData.value.v_objc.type = module::MarkerOBJC_t::OBJ_UNKNOWN;
		} else if(track_obs.type == 1) {
			m_markerData.value.v_objc.type = module::MarkerOBJC_t::OBJ_STOP;
		} else if(track_obs.type == 2) {
			m_markerData.value.v_objc.type = module::MarkerOBJC_t::OBJ_MOVE;
		} else {

		}
		m_markerData.value.v_objc.vel = sqrt(pow(track_obs.velocity.x, 2) + pow(track_obs.velocity.y, 2));
		m_markerData.value.v_objc.dis = sqrt(pow(track_obs.anchor_pt.x, 2) + pow(track_obs.anchor_pt.y, 2));

		m_markerData.value.v_objc.isSD = 1;
		m_markerData.value.v_objc.velSD = 20.0 / 3.6;

	} else {
		m_markerData.value.v_objc.type = module::MarkerOBJC_t::OBJ_UNKNOWN;
		m_markerData.value.v_objc.vel = 0;
		m_markerData.value.v_objc.dis = 80;

		m_markerData.value.v_objc.isSD = 0;
		m_markerData.value.v_objc.velSD = 20.0 / 3.6;
	}
	
	m_markerData.value.v_objc.isvalid = flag;
	//m_markerData.value.v_objc.type = track_obs.type;
	//m_markerData.value.v_objc.vel = sqrt(pow(track_obs.velocity.x, 2) + pow(track_obs.velocity.y, 2));
	//m_markerData.value.v_objc.dis = sqrt(pow(track_obs.anchor_pt.x, 2) + pow(track_obs.anchor_pt.y, 2));
	cout << "wangzhy:" << m_markerData.value.v_objc.isSD << endl;
	// m_markerData.value.v_objc.isSD = 100;
	module::shm::SHARED_OBJECTS.SetMarker(m_markerData);

}
void ObjectList::luxstop()
{
	module::MarkerData_t m_markerdata;
	m_markerdata.type = module::MarkerData_t::MARKER_BEHAVIOR;
	module::shm::SHARED_OBJECTS.GetMarker(&m_markerdata);
	if(m_markerdata.value.v_behavior.Behavior == module::MarkerBehavior_t::BEHAVIOR_CROSS||
	m_markerdata.value.v_behavior.Behavior == module::MarkerBehavior_t::BEHAVIOR_STOPLINE_PAUSE){
		module::MarkerData_t data;
		data.type = module::MarkerData::MARKER_OBSTACLE_LUX; // Lux
		data.value.v_obstacleLux.level = -1;
		module::shm::SHARED_OBJECTS.SetMarker(data);
		return;
	}
	if(HaveClosestObj && closest_obj.belief >= 8) {
		double dis = sqrt(pow(closest_obj.anchor_pt.x,2)+pow(closest_obj.anchor_pt.y,2));
		if(dis < 15) {
			module::MarkerData_t data;
			data.type = module::MarkerData::MARKER_OBSTACLE_LUX; // Lux
			data.value.v_obstacleLux.level = 3;
			module::shm::SHARED_OBJECTS.SetMarker(data);
		} else {
			module::MarkerData_t data;
			data.type = module::MarkerData::MARKER_OBSTACLE_LUX; // Lux
			data.value.v_obstacleLux.level = -1;
			module::shm::SHARED_OBJECTS.SetMarker(data);
		}
	} else {
		module::MarkerData_t data;
		data.type = module::MarkerData::MARKER_OBSTACLE_LUX; // Lux
		data.value.v_obstacleLux.level = -1;
		module::shm::SHARED_OBJECTS.SetMarker(data);
	}
}

/*
void ObjectList::saveObjects(string filename)
{
	outfile.open(filename.c_str(), ofstream::out | ofstream::app);

	for(map<unsigned int, Object>::iterator map_iter = objs.begin();
			map_iter != objs.end(); map_iter++) {
		outfile << frame << " ";
		outfile << map_iter->first << " ";
		outfile << ugv_pt.x << " " << ugv_pt.y << " "
			<< sqrt(pow(map_iter->second.sum_shift.x, 2) + pow(map_iter->second.sum_shift.y, 2)) << " "
			<< getUGVLocalPoint(map_iter->second.sum_shift).x << " " << getUGVLocalPoint(map_iter->second.sum_shift).y << " "
			<< sqrt(pow(map_iter->second.velocity.x, 2)  + pow(map_iter->second.velocity.y, 2)) << " "
			<< getUGVLocalPoint(map_iter->second.velocity).x << " " << getUGVLocalPoint(map_iter->second.velocity).y << " "
			<< endl;
	}
	outfile.close();

}
*/
