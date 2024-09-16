cmake_minimum_required(VERSION 3.8)
project(ComputationalGeometry)

# Do not generate ZERO_BUILD project.

set(CMAKE_SUPPRESS_REGENERATION ON)

# Visual Studio: Win32 or x64?

set(PATH_64 "")
if (${CMAKE_VS_PLATFORM_NAME} STREQUAL "x64")
  set(PATH_64 "/x64")
else()
  set(USE_OPEN_CV OFF CACHE BOOL "Use OpenCV? Requires 64-bit configuration." FORCE)
endif()

# Require at least C++ 14.

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# CMake variables. Options to include various libraries.

set(OPEN_CV_VERSION_STR "")
set(USE_OPEN_GL ON CACHE BOOL "Use OpenGL? Required to be true." FORCE)
set(USE_OPEN_CV OFF CACHE BOOL "Use OpenCV? Optional.")
set(USE_CUDA OFF CACHE BOOL "Use CUDA? Optional.")
if (${USE_CUDA})
  enable_language(CUDA)
  set(CUDA_VERBOSE_BUILD ON)
  #set(NVCC_PATH "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6/bin/nvcc.exe" CACHE PATH "Location of nvcc binary.")
endif()
if (${USE_OPEN_GL})
  set(FREE_GLUT_INCLUDE "freeglut/include" CACHE PATH "Location of FreeGlut headers.")
  set(FREE_GLUT_LIB_DIR "freeglut/lib${PATH_64}" CACHE PATH "Location of FreeGlut static libraries.")
  set(FREE_GLUT_BIN_DIR "freeglut/bin${PATH_64}" CACHE PATH "Location of FreeGlut DLLs.")
endif()
if (${USE_OPEN_CV})
  set(OPEN_CV_VERSION "4.9.0" CACHE STRING "Which version of OpenCV to use?")
  if ("${OPEN_CV_VERSION}" STREQUAL "4.9.0")
    set(OPEN_CV_VERSION_STR "490")
  else()
    message(WARNING "Open CV version ${OPEN_CV_VERSION} not supported.")
  endif()
  set(OPEN_CV_INCLUDE "opencv/build/include" CACHE PATH "Location of OpenCV headers.")
  set(OPEN_CV_LIB_DIR "opencv/build${PATH_64}/vc16/lib" CACHE PATH "Location of OpenCV static libraries.")
  set(OPEN_CV_BIN_DIR "opencv/build${PATH_64}/vc16/bin" CACHE PATH "Location of OpenCV DLLs.")
endif()

set(IMG_OUTPUT_FOLDER "../../image_output" CACHE PATH "Where should image outputs be written?")
set(VIDEO_OUTPUT_FOLDER "../../video_output" CACHE PATH "Where should video outputs be written?")

file(MAKE_DIRECTORY "${IMG_OUTPUT_FOLDER}")
file(MAKE_DIRECTORY "${VIDEO_OUTPUT_FOLDER}")

# Header files.

set(HEADER_FILES
  ../create_video.h
  ../edge_list.h
  ../gl_callbacks.h
  ../includes.h
  ../point_cloud.h
  ../primitives.h
  ../rational.h
)

# Source files.

set(SOURCE_FILES
  ../create_video.cpp
  ../edge_list.cpp
  ../geometry_suite.cpp
  ../gl_callbacks.cpp
  ../point_cloud.cpp
  ../primitives.cpp
  ../rational.cpp
)

# Binary output.

add_executable(ComputationalGeometry ${HEADER_FILES} ${SOURCE_FILES})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ComputationalGeometry)

# Includes.

if (${USE_OPEN_GL})
  target_include_directories(ComputationalGeometry PUBLIC "${FREE_GLUT_INCLUDE}")
endif()

if (${USE_OPEN_CV})
  target_include_directories(ComputationalGeometry PUBLIC "${OPEN_CV_INCLUDE}")
endif()

# Compiler flags.

if (${USE_CUDA})
  set_source_files_properties(../point_cloud.cpp PROPERTIES LANGUAGE CUDA)
endif()

# Linking.

if (${USE_OPEN_GL})
  target_link_directories(ComputationalGeometry PUBLIC "${FREE_GLUT_LIB_DIR}")
  target_link_libraries(ComputationalGeometry PUBLIC "freeglut.lib")
endif()

if (${USE_OPEN_CV})
  target_link_directories(ComputationalGeometry PUBLIC "${OPEN_CV_LIB_DIR}")
  target_link_libraries(ComputationalGeometry PUBLIC "opencv_world${OPEN_CV_VERSION_STR}.lib")
endif()

# Binary dependencies.

if (${USE_OPEN_GL})
  add_custom_command(TARGET ComputationalGeometry POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${FREE_GLUT_BIN_DIR}/freeglut.dll" "$<TARGET_FILE_DIR:ComputationalGeometry>")
endif()

if (${USE_OPEN_CV})
  add_custom_command(TARGET ComputationalGeometry POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${OPEN_CV_BIN_DIR}/opencv_world${OPEN_CV_VERSION_STR}.dll" "$<TARGET_FILE_DIR:ComputationalGeometry>")
  add_custom_command(TARGET ComputationalGeometry POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${OPEN_CV_BIN_DIR}/opencv_world${OPEN_CV_VERSION_STR}d.dll" "$<TARGET_FILE_DIR:ComputationalGeometry>")
  add_custom_command(TARGET ComputationalGeometry POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${OPEN_CV_BIN_DIR}/opencv_world${OPEN_CV_VERSION_STR}d.pdb" "$<TARGET_FILE_DIR:ComputationalGeometry>")
endif()

# Preprocessor Definitions.

if (${USE_OPEN_GL})
  add_definitions(-DUSE_OPEN_GL)
endif()

if (${USE_OPEN_CV})
  add_definitions(-DUSE_OPEN_CV)
endif()

if (${USE_CUDA})
  add_definitions(-DUSE_CUDA)
endif()

add_definitions(-DIMG_OUTPUT_FOLDER="${IMG_OUTPUT_FOLDER}")
add_definitions(-DVIDEO_OUTPUT_FOLDER="${VIDEO_OUTPUT_FOLDER}")
