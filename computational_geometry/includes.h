/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef INCLUDES_H
#define INCLUDES_H
#include <vector>
#include <set>
#include <string>
#include <iostream>
#include <fstream>

#include <stdlib.h>     // randomness functions - srand and rand
#include <time.h>       // time function
#include <math.h>       // atan2 function

/*  OpenGL */
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#else
#ifdef _WIN64
#include <windows.h>
#include <GL/gl.h>
#include <GL/glut.h>
#else
#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#include <GL/glut.h>
#else
#ifdef __linux__
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif // def __linux__
#endif // def _WIN32
#endif // def _WIN64
#endif // def __APPLE__

#endif // INCLUDES_H
