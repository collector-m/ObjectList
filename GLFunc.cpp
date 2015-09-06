#include "GLFunc.h"

extern ObjectList *obj_list;

int GLFunc::m_window_width = 1000;
int GLFunc::m_window_height = 1000;

int GLFunc::m_window_pos_x = 250;
int GLFunc::m_window_pos_y = 10;

float GLFunc::m_rotx = 0.0f;
float GLFunc::m_roty = 0.0f;
float GLFunc::m_rotz = 0.0f;

float GLFunc::m_tx = 0.0f;
//float GLFunc::m_ty = -500.0f;
float GLFunc::m_ty = 0.0f;
float GLFunc::m_tz = -2000.000f;
//float GLFunc::m_tz = -40000.000f;


int GLFunc::m_lastx = 0;
int GLFunc::m_lasty = 0;
unsigned char GLFunc::m_gl_buttons[3];
bool GLFunc::m_pause_screen = false;
int GLFunc::p_times = 10;

float GLFunc::shift = 1.0;
float GLFunc::click_x = 0.0;
float GLFunc::click_y = 0.0;
float GLFunc::last_click_x = 0.0;
float GLFunc::last_click_y = 0.0;

bool GLFunc::drawDetectBox = false;
bool GLFunc::drawHorDetectBox = true;
bool GLFunc::drawVerDetectBox = true;
bool GLFunc::drawLRDetectBox = false;

bool GLFunc::saveFlag = false;

void GLFunc::Draw()
{
    glColor3ub(100, 100, 100);
    //DrawRasterMap();

    DrawDetectBoxes();
    DrawVehicle();
    DrawOccupyGrids();
    DrawObjects();
    DrawAnchorPoints();

}

void GLFunc::Process(bool online)
{
    if(online) {
        obj_list->process();
		//if(saveFlag)
		//	obj_list->saveObjects("/home/denggroup/Workspace/yangguorun/ObjectList/1.txt");
		
        cout << "------------------------------------" << endl;
    } else {
        cout << "click x: " << click_x << " click y: " << click_y << endl;
        Point pt;
        pt.setPoint(click_x, click_y);
        if(!((pt.x == 0 && pt.y == 0) || (pt.x == last_click_x && pt.y == last_click_y)
                || (pt.x > 80.0) || (pt.x < -80.0) || (pt.y > 80.0) || (pt.x < -80.0) ))
            obj_list->click_pts.push_back(pt);

        obj_list->processTest();

        last_click_x = click_x;
        last_click_y = click_y;
        cout << "------------------------------------" << endl;
    }
}

void GLFunc::ProcessTest()
{
    /*
    int ugv_row = rm->ugv_row;
    int ugv_col = rm->ugv_col;
    if((ugv_row >= 0) && (ugv_row < OUTPUT_MAP_ROWS) && (ugv_col >= 0) && (ugv_col < OUTPUT_MAP_COLS)) {
        if((ugv_row >= rm->arr_dr) && (ugv_row <= rm->arr_ur) && (ugv_col >= rm->arr_lc) && (ugv_col <= rm->arr_rc))
            rm->g[ugv_row][ugv_col].test_occpy = false;
    }

    int row = rm->getRow(click_x, click_y);
    int col = rm->getCol(click_x, click_y);
    cout << "click row: " << row << " click col: " << col << endl;
    if((row >= 0) && (row < OUTPUT_MAP_ROWS) && (col >= 0) && (col < OUTPUT_MAP_COLS)) {
        if((row >= rm->arr_dr) && (row <= rm->arr_ur) && (col >= rm->arr_lc) && (col <= rm->arr_rc))
            rm->g[row][col].test_occpy = true;
    }

    last_click_x = click_x;
    last_click_y = click_y;
    rm->processTest();
    cout << "------------------------------------------" << endl;
    */
}

void GLFunc::DrawBox(float xmin, float xmax, float ymin, float ymax)
{
    glColor3ub(139, 0, 255);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(2.0);
    glBegin(GL_QUADS);
        glVertex2f(p_times * xmin, p_times * ymin);
        glVertex2f(p_times * xmax, p_times * ymin);
        glVertex2f(p_times * xmax, p_times * ymax);
        glVertex2f(p_times * xmin, p_times * ymax);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLineWidth(1.0);
}

void GLFunc::DrawOccupyGrids()
{
    glColor3ub(255, 0, 0);
    for(vector<cluster_pos_t>::iterator pos_iter = obj_list->local_convex_pos.begin();
            pos_iter != obj_list->local_convex_pos.end(); pos_iter++) {
        int row = pos_iter->row;
        int col = pos_iter->col;
        DrawOrthoGrid(row, col);
    }
}

void GLFunc::DrawVehicle()
{
    glColor3ub(0, 0, 255);
    float x1 = obj_list->ugv_box.frozen_ld_pt.x;
    float y1 = obj_list->ugv_box.frozen_ld_pt.y;
    float x2 = obj_list->ugv_box.frozen_rd_pt.x;
    float y2 = obj_list->ugv_box.frozen_rd_pt.y;
    float x3 = obj_list->ugv_box.frozen_ru_pt.x;
    float y3 = obj_list->ugv_box.frozen_ru_pt.y;
    float x4 = obj_list->ugv_box.frozen_lu_pt.x;
    float y4 = obj_list->ugv_box.frozen_lu_pt.y;
    DrawVertexBox(x1, y1, x2, y2, x3, y3, x4, y4);
}

void GLFunc::DrawDetectBoxes()
{
    //glLineWidth(2.0f);
    glColor3ub(0, 130, 0);
    if(drawDetectBox) {
        float x1 = obj_list->db.frozen_ld_pt.x;
        float y1 = obj_list->db.frozen_ld_pt.y;
        float x2 = obj_list->db.frozen_rd_pt.x;
        float y2 = obj_list->db.frozen_rd_pt.y;
        float x3 = obj_list->db.frozen_ru_pt.x;
        float y3 = obj_list->db.frozen_ru_pt.y;
        float x4 = obj_list->db.frozen_lu_pt.x;
        float y4 = obj_list->db.frozen_lu_pt.y;
        DrawVertexBox(x1, y1, x2, y2, x3, y3, x4, y4);
    }
    if(drawVerDetectBox) {
        float x1 = obj_list->ver_db.frozen_ld_pt.x;
        float y1 = obj_list->ver_db.frozen_ld_pt.y;
        float x2 = obj_list->ver_db.frozen_rd_pt.x;
        float y2 = obj_list->ver_db.frozen_rd_pt.y;
        float x3 = obj_list->ver_db.frozen_ru_pt.x;
        float y3 = obj_list->ver_db.frozen_ru_pt.y;
        float x4 = obj_list->ver_db.frozen_lu_pt.x;
        float y4 = obj_list->ver_db.frozen_lu_pt.y;
        DrawVertexBox(x1, y1, x2, y2, x3, y3, x4, y4);
    }
    if(drawHorDetectBox) {
        float x1 = obj_list->horLeft_db.frozen_ld_pt.x;
        float y1 = obj_list->horLeft_db.frozen_ld_pt.y;
        float x2 = obj_list->horLeft_db.frozen_rd_pt.x;
        float y2 = obj_list->horLeft_db.frozen_rd_pt.y;
        float x3 = obj_list->horLeft_db.frozen_ru_pt.x;
        float y3 = obj_list->horLeft_db.frozen_ru_pt.y;
        float x4 = obj_list->horLeft_db.frozen_lu_pt.x;
        float y4 = obj_list->horLeft_db.frozen_lu_pt.y;
        DrawVertexBox(x1, y1, x2, y2, x3, y3, x4, y4);

        x1 = obj_list->horRight_db.frozen_ld_pt.x;
        y1 = obj_list->horRight_db.frozen_ld_pt.y;
        x2 = obj_list->horRight_db.frozen_rd_pt.x;
        y2 = obj_list->horRight_db.frozen_rd_pt.y;
        x3 = obj_list->horRight_db.frozen_ru_pt.x;
        y3 = obj_list->horRight_db.frozen_ru_pt.y;
        x4 = obj_list->horRight_db.frozen_lu_pt.x;
        y4 = obj_list->horRight_db.frozen_lu_pt.y;
        DrawVertexBox(x1, y1, x2, y2, x3, y3, x4, y4);
    }
    if(drawLRDetectBox) {
        float x1 = obj_list->left_db.frozen_ld_pt.x;
        float y1 = obj_list->left_db.frozen_ld_pt.y;
        float x2 = obj_list->left_db.frozen_rd_pt.x;
        float y2 = obj_list->left_db.frozen_rd_pt.y;
        float x3 = obj_list->left_db.frozen_ru_pt.x;
        float y3 = obj_list->left_db.frozen_ru_pt.y;
        float x4 = obj_list->left_db.frozen_lu_pt.x;
        float y4 = obj_list->left_db.frozen_lu_pt.y;
        DrawVertexBox(x1, y1, x2, y2, x3, y3, x4, y4);

        x1 = obj_list->right_db.frozen_ld_pt.x;
        y1 = obj_list->right_db.frozen_ld_pt.y;
        x2 = obj_list->right_db.frozen_rd_pt.x;
        y2 = obj_list->right_db.frozen_rd_pt.y;
        x3 = obj_list->right_db.frozen_ru_pt.x;
        y3 = obj_list->right_db.frozen_ru_pt.y;
        x4 = obj_list->right_db.frozen_lu_pt.x;
        y4 = obj_list->right_db.frozen_lu_pt.y;
        DrawVertexBox(x1, y1, x2, y2, x3, y3, x4, y4);
    }
    //glLineWidth(1.0f);
}

void GLFunc::DrawObjects()
{
    for(map<unsigned int, Object>::const_iterator map_iter = obj_list->local_objs.begin();
            map_iter != obj_list->local_objs.end(); map_iter++) {
		if(obj_list->ver_db.InDetectBox(obj_list->local_objs[map_iter->first])) {
			float x1 = map_iter->second.bbx.vertex[0].x;
			float y1 = map_iter->second.bbx.vertex[0].y;
			float x2 = map_iter->second.bbx.vertex[1].x;
			float y2 = map_iter->second.bbx.vertex[1].y;
			float x3 = map_iter->second.bbx.vertex[2].x;
			float y3 = map_iter->second.bbx.vertex[2].y;
			float x4 = map_iter->second.bbx.vertex[3].x;
			float y4 = map_iter->second.bbx.vertex[3].y;
			if(map_iter->second.type == 1) {
				glColor3ub(139, 0, 255);
			} else if(map_iter->second.type == 2) {
				glColor3ub(255, 255, 0);
			} else {
				glColor3ub(0, 255, 255);
			}
			DrawVertexBox(x1, y1, x2, y2, x3, y3, x4, y4);
			DrawVelocity(map_iter->second.anchor_pt.x, map_iter->second.anchor_pt.y, map_iter->second.velocity.x, map_iter->second.velocity.y);
		}
	}
}

void GLFunc::DrawAnchorPoints()
{
    glColor3ub(255, 215, 0);
    glPointSize(4.0f);
    glBegin(GL_POINTS);
        for(map<unsigned int, Object>::const_iterator map_iter = obj_list->local_objs.begin();
                map_iter != obj_list->local_objs.end(); map_iter++) {
            double x = map_iter->second.anchor_pt.x;
            double y = map_iter->second.anchor_pt.y;
            glVertex2d(p_times * x, p_times * y);
        }
    glEnd();

//    glColor3ub(160, 32, 240);
//    glPointSize(6.0f);
//    glBegin(GL_POINTS);
//        for(vector<Point>::const_iterator pt_iter = obj_list->insection_pts.begin();
//                pt_iter != obj_list->insection_pts.end(); pt_iter++) {
//            double x = pt_iter->x;
//            double y = pt_iter->y;
//            glVertex2d(p_times * x, p_times * y);
//        }
//    glEnd();
//    glPointSize(1.0f);
}


void GLFunc::DrawOrthoGrid(int row, int col)
{
   glBegin(GL_QUADS);
        glVertex3f(p_times * (col-0.5) * 0.2, p_times * (row-0.5) * 0.2, 0);
        glVertex3f(p_times * (col+0.5) * 0.2, p_times * (row-0.5) * 0.2, 0);
        glVertex3f(p_times * (col+0.5) * 0.2, p_times * (row+0.5) * 0.2, 0);
        glVertex3f(p_times * (col-0.5) * 0.2, p_times * (row+0.5) * 0.2, 0);
   glEnd();
}

void GLFunc::DrawVertexBox(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
    //glColor3ub(139, 0, 255);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(2.0);
    glBegin(GL_QUADS);
        glVertex3f(p_times * x1, p_times * y1, 0);
        glVertex3f(p_times * x2, p_times * y2, 0);
        glVertex3f(p_times * x3, p_times * y3, 0);
        glVertex3f(p_times * x4, p_times * y4, 0);
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLineWidth(1.0);
}

void GLFunc::DrawVelocity(float x, float y, float v_x, float v_y, float zoom_times)
{
    double x1 = x;
    double y1 = y;
    double x2 = x + v_x * zoom_times;
    double y2 = y + v_y * zoom_times;
    glColor3ub(255, 222, 173);  //ÑÇÂéÉ«
    glLineWidth(1.0f);
    glBegin(GL_LINES);
        glVertex2d(p_times * x1, p_times * y1);
        glVertex2d(p_times * x2, p_times * y2);
    glEnd();

}

void GLFunc::DrawRasterMap()
{
    glPointSize(0.1f);
    glBegin(GL_LINES);
        //draw the front rows
        for(unsigned int i = 0; i <= 400; i++) {
            glColor3ub(100, 100, 100);
            double y = 0.2 * (i + 0.5);
            glVertex3f(-p_times * (80 + 0.5 * 0.2), p_times * y, 0);
            glVertex3f( p_times * (80 + 0.5 * 0.2), p_times * y, 0);
        }
        //draw the back rows
        for(unsigned int i = 0; i <= 400; i++) {
            glColor3ub(100, 100, 100);
            double y = -(0.2 * (i + 0.5));
            glVertex3f(-p_times * (80 + 0.5 * 0.2), p_times * y, 0);
            glVertex3f( p_times * (80 + 0.5 * 0.2), p_times * y, 0);
        }

        for(unsigned int i = 0; i <= 400; i++) {
             glColor3ub(100, 100, 100);
             double x = 0.2 * (i + 0.5);
             glVertex3f(p_times * x,  p_times * (80 + 0.5 * 0.2), 0);
             glVertex3f(p_times * x, -p_times * (80 + 0.5 * 0.2), 0);
        }
        //draw the left cols
        for(unsigned int i = 0; i <= 400; i++) {
             glColor3ub(100, 100, 100);
             double x = -(0.2 * (i + 0.5));
             glVertex3f(p_times * x,  p_times * (80 + 0.5 * 0.2), 0);
             glVertex3f(p_times * x, -p_times * (80 + 0.5 * 0.2), 0);
        }
    glEnd();
}

void GLFunc::Initialize(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowSize(GLFunc::m_window_width, GLFunc::m_window_height);
    glutInitWindowPosition(GLFunc::m_window_pos_x, GLFunc::m_window_pos_y);
    glutCreateWindow("ObjectList");
}

void GLFunc::gl_init_graphics()
{
    glClearColor(0, 0, 0, 0.5);
    //glClearColor(255, 255, 255, 0.5);
    glClearDepth(1.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void GLFunc::gl_draw_graphics()
{
    //cout << "Enter gl_draw_graphics" << endl;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glLoadIdentity();
    glTranslated(m_tx, m_ty, m_tz);

    glRotatef(m_rotx, 1, 0, 0);
    glRotatef(m_roty, 0, 1, 0);
    glRotatef(m_rotz, 0, 0, 1);

//    glBegin(GL_LINES);
//        glColor3ub( 88, 29, 29);  //dark red x axis
//        glVertex3i(0, 0, 0);
//        glVertex3i(100000, 0, 0);
//        glColor3ub( 29, 88, 29);   //dark green  z axis
//        glVertex3i(0, 0, 0);
//        glVertex3i(0, 0, 100000);
//        glColor3ub( 255, 240, 0);   //yellow  y axis
//        glVertex3i(0, 0, 0);
//        glVertex3i(0, 100000, 0);
// 	glEnd();

    glEnable(GL_BLEND);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
        Draw();
    glDisable(GL_BLEND);

    glColor3ub(255, 255, 0);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();

    glLoadIdentity();
        //gluOrtho2D(-(GLdouble)m_window_width, (GLdouble)m_window_width, -(GLdouble)m_window_height, (GLdouble)m_window_height);
        //glScalef(1, -1, 1);
        glScalef(1, 1, 1);
        glTranslatef(0, -m_window_height, 0);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
            //do something
            //
            //
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	// Show the new scene
	glutSwapBuffers();
}


void GLFunc::gl_mouse(int b, int s, int x, int y)
{
	m_lastx = x;
	m_lasty = y;
	switch(b)
	{
	case GLUT_LEFT_BUTTON:
		m_gl_buttons[0] = ((GLUT_DOWN == s) ? 1 : 0);
        cout << "x: " << x << " " << "y: " << y << endl;
		break;
	case GLUT_MIDDLE_BUTTON:
		m_gl_buttons[1] = ((GLUT_DOWN == s) ? 1 : 0);
		break;
	case GLUT_RIGHT_BUTTON:
		m_gl_buttons[2] = ((GLUT_DOWN == s) ? 1 : 0);
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void GLFunc::gl_mouse2(int button, int state, int x, int y)
{
	m_lastx = x;
	m_lasty = y;

	GLint viewport[4];
	GLdouble mvmatrix[16], projmatrix[16];
	GLint realy;    //OpenGL y coordinate position
	GLdouble wx, wy, wz;    //return world x, y, z coordinates on 33000
	//GLdouble rx, ry, rz;    //return world x, y, z coordinates on 0
    //GLdouble tx, ty;

	switch(button)
	{
        case GLUT_LEFT_BUTTON:
            m_gl_buttons[0] = ((GLUT_DOWN == state) ? 1 : 0);
            //cout << "x: " << x << " " << "y: " << y << endl;

            if(state == GLUT_DOWN) {
                glGetIntegerv(GL_VIEWPORT, viewport);
                glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
                glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);
                realy = viewport[3] - (GLint)y - 1;
                //cout << "Coordinates at cursor are ( " << x << " , " << realy << ")" << endl;

//                gluUnProject((GLdouble)x, (GLdouble)realy, 0.0, mvmatrix, projmatrix, viewport, &wx, &wy, &wz);
//                cout << "World coordinates at z = 0.0 are ( " << wx << " , " << wy << " , " << wz << ")" << endl;
                gluUnProject((GLdouble)x, (GLdouble)realy, 1.0, mvmatrix, projmatrix, viewport, &wx, &wy, &wz);
                //cout << "World coordinates at z = 30000.0 are ( " << wx << " , " << wy << " , " << wz << ")" << endl;

                //cout << "m_tx: " << m_tx << " m_ty: " << m_ty << " m_tz: " << m_tz << endl;
//                gluUnProject((GLdouble)m_tx, (GLdouble)m_ty, 1.0, mvmatrix, projmatrix, viewport, &tx, &ty, &tz);

                //compute the real x, y, z;
                //rz = 0;
                //rx = (fabs(m_tz) * wx / (30000 + fabs(m_tz))) / p_times;
                //ry = (fabs(m_tz) * wy / (30000 + fabs(m_tz))) / p_times;
//                rx = (wx * fabs(m_tz / wz)) / p_times;
//                ry = (wy * fabs(m_tz / wz)) / p_times;

//                rx = (wx * fabs(m_tz / wz) - m_tx) / p_times ;
//                ry = (wy * fabs(m_tz / wz) - m_ty) / p_times;

                click_x = (wx * fabs(m_tz / wz) - m_tx) / p_times;
                click_y = (wy * fabs(m_tz / wz) - m_ty) / p_times;

                cout << "click coordinates: " << click_x << " , " << click_y << ")" << endl;

//                cout << "World coordinates at z = 0.0 are ( " << wx << " , " << wy << " , " << wz << ")" << endl;
//                cout << "mvMatrix: ";
//                for(int i = 0; i < 16; i++)
//                    cout << mvmatrix[i] << " " ;
//                cout << endl;
//                cout << "projMatrix: ";
//                for(int i = 0; i < 16; i++)
//                    cout << projmatrix[i] << " " ;
//
//                cout << endl;
            }
            break;
        case GLUT_MIDDLE_BUTTON:
            m_gl_buttons[1] = ((GLUT_DOWN == state) ? 1 : 0);
            break;
        case GLUT_RIGHT_BUTTON:
            m_gl_buttons[2] = ((GLUT_DOWN == state) ? 1 : 0);
            break;
        default:
            break;
	}
	glutPostRedisplay();
}


void GLFunc::gl_motion(int x,int y)
{
	int diffx = x - m_lastx;
	int diffy = y - m_lasty;
	m_lastx = x;
	m_lasty = y;

    //added by Guorun
    /*
    if(m_gl_buttons[0]) {
        m_rotx += 0.4f * diffy;
		m_roty += 0.4f * diffx;
    } else if(m_gl_buttons[1]) {
        m_tz += 1000.0f;
    } else if(m_gl_buttons[2]) {
        m_tx += 15.0f * diffx;
        m_ty -= 15.0f * diffy;
    }
    */
    ///*
	if( m_gl_buttons[0] && m_gl_buttons[2] ) //transition
	{
		//m_zoom += 25.0f * diffy;
		m_tx += 25.0f * diffx;
        m_ty -= 25.0f * diffy;
	}
	else if( m_gl_buttons[0] ) //rotation
	{
		m_tx += 10.0f * diffx;
        m_ty -= 10.0f * diffy;
	}
	else if( m_gl_buttons[2] ) //transition
	{
		//m_tx += 85.0f * diffx;
		//m_ty -= 85.0f * diffy;
		m_rotx += 0.4f * diffy;
		m_roty += 0.4f * diffx;
	}
    //*/
	glutPostRedisplay();
}

void GLFunc::gl_motion2(int x,int y)
{
	int diffx = x - m_lastx;
	int diffy = y - m_lasty;
	m_lastx = x;
	m_lasty = y;


	if( m_gl_buttons[0] && m_gl_buttons[2] ) //transition
	{
		//m_zoom += 25.0f * diffy;
		m_tx += 25.0f * diffx;
        m_ty -= 25.0f * diffy;
	}
	else if( m_gl_buttons[0] ) //rotation
	{
		m_tx += 10.0f * diffx;
        m_ty -= 10.0f * diffy;

	}
	else if( m_gl_buttons[2] ) //transition
	{
		//m_tx += 85.0f * diffx;
		//m_ty -= 85.0f * diffy;
		m_rotx += 0.4f * diffy;
		m_roty += 0.4f * diffx;
	}
	glutPostRedisplay();
}


void GLFunc::gl_keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
        case '.':
            m_tz += 100.0f;
            cout << "bigger!" << endl;
            break;
        case ',':
            m_tz -= 100.0f;
            cout << "smaller!" << endl;
            break;
        case '1':
            drawDetectBox = !drawDetectBox;
            break;
        case '2':
            drawVerDetectBox = !drawVerDetectBox;
            break;
        case '3':
            drawHorDetectBox = !drawHorDetectBox;
            break;
        case '4':
            drawLRDetectBox = !drawLRDetectBox;
            break;
        case 'a':
            obj_list->delta_pose.x -= shift;
            cout << "left!" << endl;
            break;
        case 'd':
            obj_list->delta_pose.x += shift;
            cout << "right!" << endl;
            break;
        case 's':
            obj_list->delta_pose.y -= shift;
            cout << "down!" << endl;
            break;
        case 'w':
            obj_list->delta_pose.y += shift;
            cout << "up!" << endl;
            break;
        case 'k':
            obj_list->delta_pose.eulr -= 0.05f;
            cout << "Rotate: anti-clockwise!" << endl;
            break;
        case 'l':
            obj_list->delta_pose.eulr += 0.05f;
            cout << "Rotate: clockwise!" << endl;
            break;
        case 'c':
            obj_list->click_pts.clear();
            break;
		case 'r':
			saveFlag = !saveFlag;
			break;
        case 27:
            exit(0);
        case 32:
            m_pause_screen = !m_pause_screen;
            break;
        default:
            break;
	}
	glutPostRedisplay();
}

void GLFunc::gl_resize_graphics(int width, int height)
{
	cout << "width: " << width << " height: " << height << endl;
	if (width == 0)
		height = 1;
	m_window_width = width;
	m_window_height = height;

	// Adjust graphics to window size
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//gluPerspective(45, ((double)width)/height, 0.05, 400000);
    gluPerspective(45, ((double)width)/height, 0.05, 400000.0);


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void GLFunc::gl_when_idle()
{
	if (!(m_pause_screen)) {
		usleep(10000);
        Process(true);
        glutPostRedisplay();
	} else {
//        int row = rm->getRow(click_x, click_y);
//        int col = rm->getCol(click_x, click_y);
//        if((row >= 0) && (row < OUTPUT_MAP_ROWS) && (col >= 0) && (col < OUTPUT_MAP_COLS)) {
//            if((row >= rm->arr_dr) && (row <= rm->arr_ur) && (col >= rm->arr_lc) && (col <= rm->arr_rc))
//                rm->g[row][col].test_occpy = true;
//        }
		//cout << "pause" << endl;
	}
}
