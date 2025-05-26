#!/bin/bash

g++ -std=c++14 -o function_calculator.o *.cpp -framework OpenGL -framework GLUT -w
./function_calculator.o "${1}"
