/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef MESH_GL_CALLBACKS_H
#define MESH_GL_CALLBACKS_H

static int& GetWindowId();
void initialize_glut(int* argc_ptr, char** argv);
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void render();

#endif //def MESH_GL_CALLBACKS_H
