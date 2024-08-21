#!/bin/bash

#open README.pdf
# Adding -w flag to suppress all warnings because of GLUT deprecation warnings for Mac OSX.
# -std=c++20 is also possible.
g++ -std=c++14 -o geometry_suite.o *.cpp -framework OpenGL -framework GLUT -w
./geometry_suite.o "${1}"
