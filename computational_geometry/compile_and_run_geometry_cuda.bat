echo off

set numPoints=%1
if "%1" == "" set numPoints=50

mkdir comp_geo_cuda
copy ComputationalGeometry\freeglut\bin\x64\freeglut.dll comp_geo_cuda
REM Compile (and link) with CUDA compiler:
nvcc -x cu geometry_suite.cpp gl_callbacks.cpp point_cloud.cpp rational.cpp -o comp_geo_cuda\comp_geo_cuda.exe -I="ComputationalGeometry\freeglut\include" --library-path="ComputationalGeometry\freeglut\lib\x64" --library=freeglut
REM Run application.
comp_geo_cuda\comp_geo_cuda.exe %numPoints%

