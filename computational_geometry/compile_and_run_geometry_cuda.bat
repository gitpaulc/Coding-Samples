echo off

REM Allow number of points as input.
set numPoints=%1
if "%1" == "" set numPoints=50

REM Create an output folder if it does not already exist.
mkdir comp_geo_cuda
copy ComputationalGeometry\freeglut\bin\x64\freeglut.dll comp_geo_cuda

REM Compile (and link) with CUDA compiler.
REM The -x cu option tells nvcc that source files do not need to use a .cu extension.
nvcc -x cu create_video.cpp geometry_suite.cpp gl_callbacks.cpp point_cloud.cpp rational.cpp -o comp_geo_cuda\comp_geo_cuda.exe -I="ComputationalGeometry\freeglut\include" --library-path="ComputationalGeometry\freeglut\lib\x64" --library=freeglut

REM Run application.
comp_geo_cuda\comp_geo_cuda.exe %numPoints%
