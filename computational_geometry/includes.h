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
#include <GL/gl.h>
#include <GL/glut.h>
#endif // working with __APPLE__

#endif // INCLUDES_H
