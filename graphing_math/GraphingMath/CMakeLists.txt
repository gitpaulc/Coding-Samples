cmake_minimum_required(VERSION 3.8)
set(PROJ_NAME "GraphingMath")
project("${PROJ_NAME}")

# Do not generate ZERO_BUILD project.

set(CMAKE_SUPPRESS_REGENERATION ON)

# Visual Studio: Win32 or x64?

set(PATH_64 "")
if (${CMAKE_VS_PLATFORM_NAME} STREQUAL "x64")
  set(PATH_64 "/x64")
endif()

# Require at least C++ 14.

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# CMake variables. Options to include various libraries.

set(USE_OPEN_GL ON CACHE BOOL "Use OpenGL? Required to be true." FORCE)
if (${USE_OPEN_GL})
  set(FREE_GLUT_INCLUDE "freeglut/include" CACHE PATH "Location of FreeGlut headers.")
  set(FREE_GLUT_LIB_DIR "freeglut/lib${PATH_64}" CACHE PATH "Location of FreeGlut static libraries.")
  set(FREE_GLUT_BIN_DIR "freeglut/bin${PATH_64}" CACHE PATH "Location of FreeGlut DLLs.")
endif()

# Header files.

set(HEADER_FILES
  ../graphing_math.h
  ../includes.h
  ../math.h
)

# Source files.

set(SOURCE_FILES
  ../graphing_math.cpp
  ../main.cpp
  ../math.cpp
)

# Binary output.

add_executable("${PROJ_NAME}" ${HEADER_FILES} ${SOURCE_FILES})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT "${PROJ_NAME}")

# Includes.

if (${USE_OPEN_GL})
  target_include_directories("${PROJ_NAME}" PUBLIC "${FREE_GLUT_INCLUDE}")
endif()

# Compiler flags.

# Linking.

if (${USE_OPEN_GL})
  target_link_directories("${PROJ_NAME}" PUBLIC "${FREE_GLUT_LIB_DIR}")
  target_link_libraries("${PROJ_NAME}" PUBLIC "freeglut.lib")
endif()


# Binary dependencies.

if (${USE_OPEN_GL})
  add_custom_command(TARGET "${PROJ_NAME}" POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${FREE_GLUT_BIN_DIR}/freeglut.dll" "$<TARGET_FILE_DIR:${PROJ_NAME}>")
endif()

# Preprocessor Definitions.

if (${USE_OPEN_GL})
  add_definitions(-DUSE_OPEN_GL)
endif()

