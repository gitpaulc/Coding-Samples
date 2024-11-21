#!/bin/bash

#open README.md
# Adding -w flag to suppress all warnings because of GLUT deprecation warnings for Mac OSX.
# -std=c++20 is also possible.
g++ -std=c++14 -o mesh_renderer.o *.cpp -framework OpenGL -framework GLUT -w
./mesh_renderer.o "${1}"
