#!/bin/bash

clear
g++ *.cpp -Wno-c++11-extensions -o output.o
if [[ -a output.o ]] ; then
  ./output.o
fi
