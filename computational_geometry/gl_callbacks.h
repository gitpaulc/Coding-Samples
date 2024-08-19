/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef GL_CALLBACKS_H
#define GL_CALLBACKS_H

#include "includes.h"

static int& GetWindowId();
void initialize_glut(int* argc_ptr, char** argv);
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void render();

#endif //def GL_CALLBACKS_H
