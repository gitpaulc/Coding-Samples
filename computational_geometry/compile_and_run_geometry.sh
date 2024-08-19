#!/bin/bash

#open README.pdf
# Adding -w flag to suppress all warnings because of GLUT deprecation warnings for Mac OSX.
g++ -o geometry_suite.o geometry_suite.cpp -framework OpenGL -framework GLUT -w
./geometry_suite.o "${1}"