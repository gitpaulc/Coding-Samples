#!/bin/bash

g++ -std=c++14 -I private_headers/.. -I private_headers -o function_calculator.o *.cpp -framework OpenGL -framework GLUT -w
./function_calculator.o "${1}"
