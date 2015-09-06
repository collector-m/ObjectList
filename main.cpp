#include <iostream>

#include "GLFunc.h"
#include "ObjectList.h"

using namespace std;

ObjectList *obj_list = NULL;

int main(int argc, char** argv)
{
    obj_list = new ObjectList();

    obj_list->getSHMBasePoint();

    //visualization version
    /*
	GLFunc::Initialize(argc, argv);
	glutDisplayFunc(GLFunc::gl_draw_graphics);
	glutReshapeFunc(GLFunc::gl_resize_graphics);
	glutMouseFunc(GLFunc::gl_mouse2);
	glutMotionFunc(GLFunc::gl_motion2);
	glutKeyboardFunc(GLFunc::gl_keyboard);
	GLFunc::gl_init_graphics();
	glutIdleFunc(GLFunc::gl_when_idle);
	glutMainLoop();
	*/
    //non-visualization version
    while(1) {
		obj_list->process();
		cout << "------------------------" << endl;
    }

    return 0;
}
